import multiprocessing as mp
import traceback
import time
import numpy as np
import platform
from multiprocessing import shared_memory
from .agent_factory import AgentFactory


def _agent_worker(conn, agent_factory: AgentFactory, allowed_methods=None, max_mem: float=4):
    if platform.system() != "Windows":
        try:
            import resource
            MAX_MEM = max_mem * 1024 * 1024 * 1024
            resource.setrlimit(resource.RLIMIT_AS, (MAX_MEM, MAX_MEM)) # type: ignore
        except ImportError:
            print('resource module import error.')
            pass

    agent = agent_factory()
    attached_shm = {}
    while True:
        try:
            msg = conn.recv()
        except EOFError:
            break

        cmd = msg[0]

        if cmd == "call":
            _, method_name, args, kwargs = msg

            try:
                if allowed_methods is not None and method_name not in allowed_methods:
                    raise AttributeError(f"method not allowed: {method_name}")

                # --- 共有メモリのインターセプト処理 ---
                if method_name == 'policy' and 'observation' in kwargs:
                    obs = kwargs['observation']
                    if 'shm_name' in obs:
                        shm_name = obs.pop('shm_name')
                        shm_shape = obs.pop('shm_shape')
                        shm_dtype = obs.pop('shm_dtype')
                        
                        # まだアタッチしていなければ接続
                        if shm_name not in attached_shm:
                            attached_shm[shm_name] = shared_memory.SharedMemory(name=shm_name)
                        
                        # 共有メモリのバッファを参照するNumPy配列を復元
                        depth_array = np.ndarray(
                            shm_shape, 
                            dtype=shm_dtype, 
                            buffer=attached_shm[shm_name].buf
                        )
                        
                        # NumPy配列オブジェクトにすり替える
                        obs['depth_map'] = depth_array
                # ----------------------------------

                method = getattr(agent, method_name)
                result = method(*args, **kwargs)
                conn.send(("ok", result))
            except Exception:
                conn.send(('error', traceback.format_exc()))

        elif cmd == 'close':
            break


class TimedAgentRunner:
    def __init__(self, agent_factory: AgentFactory, allowed_methods=None, max_mem: float=4, verbose: bool = True):
        self.ctx = mp.get_context('spawn')
        self.agent_factory = agent_factory
        self.allowed_methods = allowed_methods
        self.verbose = verbose
        self.max_mem = max_mem

        self.parent_conn = None
        self.proc = None
        self._start()


    def _start(self):
        parent_conn, child_conn = self.ctx.Pipe()
        proc = self.ctx.Process(
            target=_agent_worker,
            args=(child_conn, self.agent_factory, self.allowed_methods, self.max_mem),
            daemon=True,
        )
        proc.start()

        child_conn.close()

        self.parent_conn = parent_conn
        self.proc = proc


    def _restart(self):
        self.close()
        self._start()


    def call(self, method_name: str, time_out_sec: float, fallback=None, restart_on_timeout: bool = True, *args, **kwargs):
        """
        agent.method_name(*args, **kwargs) を timeout 付きで呼ぶ。
        timeout 時は fallback を返す。

        fallback:
          - 値そのもの
          - callable の場合 fallback(*args, **kwargs) として呼ぶ
        """
        start = time.perf_counter()
        if self.parent_conn is not None:
            self.parent_conn.send(('call', method_name, args, kwargs))
        if self.parent_conn is not None and self.parent_conn.poll(time_out_sec): 
            elapsed = time.perf_counter() - start
            status, payload = self.parent_conn.recv()
            if status == 'ok':
                if self.verbose:
                    print(f'[TimedAgentRunner] ({method_name}) success: {elapsed:.3f}s, {time_out_sec}')
                return payload, elapsed
            raise RuntimeError(f"error in '{method_name}':\n{payload}")

        # timeout
        elapsed = time.perf_counter() - start
        if method_name in {'get_init_states'}:
            raise TimeOutError(f"[TimedAgentRunner] ('{method_name}' or '__init__') timeout: " f"limit={time_out_sec:.3f}s, elapsed={elapsed:.3f}s")

        if self.verbose:
            print(f"[TimedAgentRunner] ({method_name}) timeout: " f"limit={time_out_sec:.3f}s, elapsed={elapsed:.3f}s")
        result = fallback(*args, **kwargs) if callable(fallback) else fallback


        if restart_on_timeout:
            self._restart()

        return result, elapsed


    def close(self):
        if self.parent_conn is not None:
            try:
                self.parent_conn.send(('close',))
            except Exception:
                pass
            try:
                self.parent_conn.close()
            except Exception:
                pass

        if self.proc is not None:
            if self.proc.is_alive():
                self.proc.kill()
            self.proc.join(timeout=1)

        self.parent_conn = None
        self.proc = None
        print('closed runner.')


class TimeOutError(Exception):
    pass
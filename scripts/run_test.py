from argparse import ArgumentParser
from src.ground_handling.app import EvaluationApp


def parse_args():
    parser = ArgumentParser()
    parser.add_argument('--config-path', default='configs/sample_config.json', help='config path', type=str)
    parser.add_argument('--module-path', default='agents/base/', help='agent module path', type=str)
    parser.add_argument('--result-dir', default = 'results/', help = 'result dir', type=str)
    parser.add_argument('--result-fname', default = 'evaluation_results.json', help = 'result fname', type=str)
    parser.add_argument('--render-mode', default = None, help = 'render mode(human or None)', type=str)
    parser.add_argument('--verbose', default = False, help = 'verbose', type=bool)

    return parser.parse_args()


# プロジェクトルートで実行
def main():
    args = parse_args()
    module_path = args.module_path # 相対パス
    agent_module_path = '.'.join(module_path.split('/'))+'agent' # モジュール名は"agent"で固定
    config_path = args.config_path
    result_dir = args.result_dir
    result_fname = args.result_fname

    runner = EvaluationApp(config_path = config_path,
                           module_path = module_path, 
                           agent_module = agent_module_path,
                           agent_class = 'Agent',
                           result_dir = result_dir,
                           result_fname = result_fname)
    runner.run(render_mode=args.render_mode, verbose=args.verbose)


if __name__ == '__main__':
    main()

from importlib import import_module
from . import Agent


class AgentFactory:
    def __init__(self, module_name: str, class_name: str, **agent_kwargs):
        self.module_name = module_name
        self.class_name = class_name
        self.agent_kwargs = agent_kwargs


    def __call__(self) -> Agent:
        module = import_module(self.module_name)
        agent_cls = getattr(module, self.class_name)

        return agent_cls(**self.agent_kwargs)

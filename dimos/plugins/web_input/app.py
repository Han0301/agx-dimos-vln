"""Compatibility factory for the VLN application; generic HTTP IO is in transport."""


def create_app(config, runner_factory=None, *, llm_preload_timeout_s=None):
    from dimos.vln_task.web import create_app as create_task_app

    return create_task_app(config, runner_factory, llm_preload_timeout_s=llm_preload_timeout_s)

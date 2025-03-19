import yaml
import os
from .dotdict import DotDict
import rospkg


def get_dict(model_name):
    model, tire = model_name.split("_")
    package_path = "/home/nicolaslauzon/ws/vaul/sim_ws/src/On-Track-SysID"
    with open(f"{package_path}/models/{model}/{model_name}.txt", "rb") as f:
        params = yaml.load(f, Loader=yaml.Loader)

    return params


def get_dotdict(model_name):
    dict = get_dict(model_name)
    params = DotDict(dict)
    return params


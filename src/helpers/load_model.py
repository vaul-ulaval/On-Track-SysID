import yaml
import os
from .dotdict import DotDict

def get_dict(model_name):
    model, tire = model_name.split("_")
    path = os.path.dirname(os.path.abspath(__file__))
    with open(f'{path}/../models/{model}/{model_name}.txt', 'rb') as f:
        params = yaml.load(f, Loader=yaml.Loader)
    
    return params

def get_dotdict(model_name):
    dict = get_dict(model_name)
    params = DotDict(dict)
    return params
import yaml
import os


def save(model, overwrite_existing=True, verbose=False):

    package_path = "/home/nicolaslauzon/ws/vaul/sim_ws/src/On-Track-SysID"
    file_path = (
        package_path
        + "/models/"
        + model["model_name"]
        + "/"
        + model["model_name"]
        + "_"
        + model["tire_model"]
        + ".txt"
    )
    if os.path.isfile(file_path):
        if verbose:
            print("Model already exists")
        if overwrite_existing:
            if verbose:
                print("Overwriting...")
        else:
            if verbose:
                print("Not overwriting.")
            return 0

    try:
        model = model.to_dict()
    except:
        model = model

    # Write data to the file
    with open(file_path, "w") as f:
        yaml.dump(model, f, default_flow_style=False)


#!/usr/bin/python3

import sys
import os
import argparse
from copy import deepcopy

from iii_drone_configuration.parameter_handler import ParameterHandler

def validate_parameter_file_name(file_name: str) -> bool:
    """
    Validates the parameter file name, checks that it is a yaml file.

    Parameters:
        file_name (str): File name.

    Returns:
        bool: True if the file name is valid, False otherwise.
    """

    if file_name[-5:] != ".yaml":
        return False
    
    if file_name[0] == ".":
        return False
    
    if "/" in file_name:
        return False
    
    if "\\" in file_name:
        return False
    
    if file_name == "":
        return False
    
    return True

def main():
    parser = argparse.ArgumentParser(description='Update installed parameters')
    parser.add_argument('parameter_file', metavar="FILE", type=str, help='Parameter file to update from')
    parser.add_argument('parameter_install_dir', metavar="DIR", type=str, help='Installed parameters directory')
    parser.add_argument('--remove-legacy-parameters', help='Remove legacy parameters from installed parameter files', action='store_true', default=True)
    parser.add_argument('--overwrite-existing-parameters', help='Overwrite existing parameters in installed parameter files', action='store_true', default=True)
    args = parser.parse_args()

    update_ph = ParameterHandler.from_parameter_file(args.parameter_file)
    
    param_files = [
        os.path.join(
            args.parameter_install_dir, 
            f
        ) for f in os.listdir(args.parameter_install_dir) 
        if os.path.isfile(
            os.path.join(
                args.parameter_install_dir, 
                f
            )
        ) and validate_parameter_file_name(f)
    ]
    
    all_update_params = update_ph.get_all_params()
    all_update_param_names = list(all_update_params.keys())

    if len(param_files) == 0:
        # Copy parameter file to directory:
        print(f"Copying {args.parameter_file} to {args.parameter_install_dir}")
        os.system(f"cp {args.parameter_file} {args.parameter_install_dir}")
        
        return

    for param_file in param_files:
        print(f"Updating {param_file}")
        
        update_param_names = deepcopy(all_update_param_names)
        
        ph = ParameterHandler.from_parameter_file(param_file)
        
        target_params = ph.get_all_params()
        target_param_names = list(target_params.keys())
        
        for update_param_name in all_update_param_names:
            update_param = all_update_params[update_param_name]

            if update_param_name in target_param_names:
                try:
                    ph.update_param(
                        update_param_name, 
                        update_param,
                        keep_value=not args.overwrite_existing_parameters
                    )
                except Exception as e:
                    print(f"Failed to update parameter: " + str(e))
                    print(f"Parameter name: {update_param_name}")
                    print(f"Parameter dict: {update_param}")
                    print(f"Parameter file: {param_file}")
                    exit(1)
                
                update_param_names.remove(update_param_name)
                target_param_names.remove(update_param_name)
                
            else:
                try:
                    ph.add_param(update_param_name, update_param)
                except Exception as e:
                    print(f"Failed to add parameter: " + str(e))
                    print(f"Parameter name: {update_param_name}")
                    print(f"Parameter dict: {update_param}")
                    print(f"Parameter file: {param_file}")
                    exit(1)

                update_param_names.remove(update_param_name)
                
        if args.remove_legacy_parameters:
            for target_param_name in target_param_names:
                try:
                    ph.remove_param(target_param_name)
                except Exception as e:
                    print(f"Failed to remove parameter: " + str(e))
                    print(f"Parameter name: {target_param_name}")
                    print(f"Parameter file: {param_file}")
                    exit(1)
                    
        try:
            ph.validate()
        except Exception as e:
            print(f"Failed to validate parameter file: " + str(e))
            print(f"Parameter file: {param_file}")
            exit(1)
            
        ph.save_parameters(
            param_file,
            overwrite=True
        )
    
if __name__ == '__main__':
    main()
##### OTHER IMPORTS
import subprocess
    
##### XACRO FUNCTION (inputs xacro file outputs urdf string)
def XacroToURDF(filepath: str, args: dict) -> str:
    print(f'\n=====PARSING XACRO FILE=====')

    # formatting arguments for xacro
    formatted_args = [f' {name}:={value}' for name, value in args.items()]

    cmd = ['xacro', filepath, *formatted_args] # base command line

    # running result
    rawurdf = subprocess.run(cmd, capture_output=True, text=True, check=True)

    return rawurdf.stdout # return urdf string

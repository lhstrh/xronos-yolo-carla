import os
import glob
import re

def remove_code_snippet(directory, snippet_regex):
    # Navigate to the specified directory and list all files recursively
    os.chdir(directory)
    files = glob.glob('**/*', recursive=True)

    for file in files:
        # Check if the current file is a regular file (not a directory, etc.)
        if os.path.isfile(file):
            try:
                with open(file, 'r+', encoding='utf-8') as f:
                    content = f.read()
                    # Use regular expression to remove the specified code snippet
                    modified_content = re.sub(snippet_regex, '', content, flags=re.MULTILINE)
                    
                    # If modifications were made, write back to the file
                    if modified_content != content:
                        f.seek(0)  # Go back to the start of the file
                        f.write(modified_content)
                        f.truncate()  # Remove any remaining original content after the new end
            except UnicodeDecodeError:
                continue
# Regular expression pattern to match and remove
print("Disabling Checkers...")
snippet_regex = r"all\(isinstance\(v, bytes\) for v in value\) and"

# Replace 'path_to_install_folder' with the actual path to the 'install/' folder
path_to_install_folder = 'install'

remove_code_snippet(path_to_install_folder, snippet_regex)

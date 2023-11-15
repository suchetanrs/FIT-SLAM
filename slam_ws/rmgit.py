import os
import subprocess

def remove_git_folders(directory):
    for root, dirs, _ in os.walk(directory):
        # Check if .git and .github folders exist in the current directory
        if '.git' in dirs:
            git_folder = os.path.join(root, '.git')
            subprocess.run(['rm', '-rf', git_folder])
            print(f"Removed {git_folder}")
        
        if '.github' in dirs:
            github_folder = os.path.join(root, '.github')
            subprocess.run(['rm', '-rf', github_folder])
            print(f"Removed {github_folder}")

if __name__ == "__main__":
    current_directory = os.getcwd()
    remove_git_folders(current_directory)
# TODO:


# Getting Started with USB CAN bus

To check if USB CAN bus is working, run

```
ifconfig -a
```

and verify that `can0` appears in the list.

# Install Code base

Do this by running the installation.sh file alternatively, you can manually run the below commands yourself

## Cloning Repo

```
sudo apt update
sudo apt-get install git
git clone https://github.com/GioByte10/Hexapod-Code.git
```

## Installing Packages

Install can utils:

```
sudo apt-get install can-utils
```

Install pip packages:

```
pip install pyinstrument
pip install numpy
pip install matplotlib
pip install python-can
```

## Add Python to ubuntu path

- type sudo gedit ~/.bashrc in /home directory 
- Insert this command near end of text file: 
  '''
    export PATH="$HOME/Desktop/Hexapod-Code:$PATH"
  '''
- Run source ~/.bashrc
- save and verify by running echo $PATH in terminal 

## Otherwise
```Python
# Add at the top of the file
from os.path import dirname, realpath  
import sys  
arcsnake_v2_path = dirname(dirname(realpath(__file__)))  
sys.path.append(arcsnake_v2_path)
``` 

## Running test programs
cd inside of arcsnake directory 
run 

```
cd Hexapod-Code/tests/System Tests
python3 pent_test.py
```

# GitHub Setup
Follow these steps to be able to contribute to the repository

## Configure Identity
```
git config --global user.name "your-github-username"
git config --global user.email "your-github-email@example.com"
```

## Caching Credentials
Make sure `curl` and `apt-transport-https` are installed
```
sudo apt update
sudo apt install curl apt-transport-https
```

Get GitHub CLI package from its repository
```
curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo tee /usr/share/keyrings/githubcli-archive-keyring.gpg > /dev/null
echo "deb [signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null
```

Install it
```
sudo apt update
sudo apt install gh
```

Verify installation
```
gh --version
```

Cache credentials
```
gh auth login
```

Select `GitHub.com`, `HTTPS`, `Login with a web browser`. Go to [hhtps://github.com/login/device](hhtps://github.com/login/device) on another device (e.g., computer) and type the code

sudo apt-get install sshpass

echo "alias sshjackal='sshpass -p 'clearpath' ssh administrator@192.168.131.11'" >> ~/.bashrc
echo "alias sshpelican='sshpass -p 'asctec' ssh asctec@192.168.131.207'" >> ~/.bashrc
source ~/.bashrc

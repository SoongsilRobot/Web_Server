
<img width="981" height="558" alt="image" src="https://github.com/user-attachments/assets/ae253110-b621-49e4-8554-89922ec4fc9b" />


# Web_Server
This is Server

1. Git clone
git clone git@github.com:SoongsilRobot/Web_Server.git
cd Web_Server

or 
git clone https://github.com/SoongsilRobot/Web_Server.git
cd Web_Server

2. Create virtual Enviroment
sudo apt update
sudo apt install python3-venv -y
python3 -m venv venv

3. Activate virtual Enviromanet
source venv/bin/activate

4. install Python Package
python -m pip install --upgrade pip --break-system-packages
pip install -r requirements.txt --break-system-packages

5. Activate ROS 2 Jazzy Setting
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

6. Run FastAPI Server
source venv/bin/activate
python run_server.py

And After Started, you can enter 

How to know I access
Swagger: http://localhost:8000/docs
Outer Acceess: http://<Raspi_IP>:8000/docs

<img width="806" height="449" alt="image" src="https://github.com/user-attachments/assets/05999c37-29ce-4894-a7c9-da1110e7266b" />


SSH인증이 필요할 때 -GIT
ssh-keygen -t ed25519 -C "your@email.com"
cat ~/.ssh/id_ed25519.pub

from fastapi import APIRouter
from app.command_model import Command
from app.ros_interface import ros

router = APIRouter()

@router.post("/command", summary="Send robot command")
def send_command(cmd: Command):
    msg = f"AXIS:{cmd.axis} DIR:{cmd.direction} DEG:{cmd.degree}"
    ros.send_command(msg)
    return {"status": "sent", "command": msg}

@router.get("/feedback", summary="Get latest feedback from ROS")
def get_feedback():
    feedback = ros.get_latest_feedback()
    return {"feedback": feedback or "no data"}
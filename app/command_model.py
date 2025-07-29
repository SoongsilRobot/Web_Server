from pydantic import BaseModel

class Command(BaseModel):
    axis: int
    direction: str
    degree: float
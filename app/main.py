from fastapi import FastAPI
from app.endpoints import router

app = FastAPI(title="Robot Control API", version="1.0")
app.include_router(router)
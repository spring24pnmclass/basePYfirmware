from fastapi import FastAPI
#from bot_components import neo6m

#gps = neo6m()
#list = gps.getLocation()
list = [37, -121]
app = FastAPI()
@app.get('/')
def root():

    return {list(0): list(1)}
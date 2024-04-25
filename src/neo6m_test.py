from bot_components import neo6m

loc = neo6m()
while True:
    location = loc.getLocation()
    print(location)
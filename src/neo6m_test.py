from togo_bot import neo6m

loc = neo6m()
while True:
    location = loc.getLocation();
    print(location)
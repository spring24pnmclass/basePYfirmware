# TODO organize inputs here. 
import forward from methods.py


"""
This should be where all control @pathfinding will focus on coding. class togo_bot will have all abstractions. 
"""
def main(): 

    '''
    1/64 encoder 
    2 * pi ~ 6 inches

    init 300 -> 620 // difference is 320
    320/64 = 5 rotations
    5 * 6 inches = 30 inches
    '''
    turnRight90()
    forward()


if __name__ == "__main__": 
    main() 

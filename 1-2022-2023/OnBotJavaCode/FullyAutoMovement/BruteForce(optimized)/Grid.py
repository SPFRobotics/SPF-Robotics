import numpy as np
import math
import pyglet

#y,x,corner(only Top/forward)
#groundPos = [(5,0,'R'),(5,1,'L'),(5,2,'R'),(5,3,'L'),(5,4,'R'),(5,5,'L'),(3,0,'R'),(3,1,'L'),(3,2,'R'),(3,3,'L'),(3,4,'R'),(3,5,'L'),(1,0,'R'),(1,1,'L'),(1,2,'R'),(1,3,'L'),(1,4,'R'),(1,5,'L')]
groundPos = [(0,0,'L'),(0,1,'R'),(0,2,'L'),(2,0,'L'),(2,1,'R'),(2,2,'L')]
#lowPos = [(5,1,'R'),(5,2,'L'),(5,3,'R'),(5,4,'L'),(4,0,'R'),(4,1,'L'),(4,4,'R'),(4,5,'L'),(2,0,'R'),(2,1,'L'),(2,4,'R'),(2,5,'L'),(1,1,'R'),(1,2,'L'),(1,3,'R'),(1,4,'L')]
lowPos = [(1,1,'R'),(1,2,'L'),(2,0,'R'),(2,1,'L')]
#midPos = [(4,1,'R'),(4,2,'L'),(4,3,'R'),(4,4,'L'),(2,1,'R'),(2,2,'L'),(2,3,'R'),(2,4,'L')]
midPos = [(1,0,'R'),(1,1,'L')]
#highPos = [(2,2,'R'),(2,3,'L'),(3,1,'R'),(3,2,'L'),(3,3,'R'),(3,4,'L'),(4,2,'R'),(4,3,'L')]
highPos = [(0,0,'R'),(0,1,'L'),(1,0,'L')]
#conePos = (3,5)
conePos = (0,2)
#yCord = [0,1,2,3,4,5]
yCord = [0,1,2]
#xCord = [0,1,2,3,4,5]
xCord = [0,1,2]
moveTime = 0.5

class Field(object):
    def __init__(self):
        #self.field = np.zeros((6,6))
        self.field = np.zeros((3,3))
        #positions in (y,x) format
        #starting pos either 5,4(bottom right)
        #starting pos 2,1(bottom left)(rotated 90 left)
        self.Time = 0
        self.Points = 0
        self.Moves = []
        self.hasCone = True
        #self.position = (5,4)
        self.position = (2,1)
    
    def compairJunction(self,y,x,side,place=False):
        allowed = 'Not Allowed'
        for junctions in groundPos:
            if (y,x,side) == junctions:
                if place == True and self.hasCone == True:
                    if side == 'L':
                        self.Time += 1.5
                    elif side == 'R':
                        self.Time += 1.5
                    self.Points += 2
                    self.hasCone = False
                    self.Moves.append('groundJunc')
                allowed = 'Allowed'
            else:
                notAllowed = 'Not Allowed'
        for junctions in lowPos:
            if (y,x,side) == junctions:
                if place == True and self.hasCone == True:
                    if side == 'L':
                        self.Time += 3
                    elif side == 'R':
                        self.Time += 3
                    self.Points += 3
                    self.hasCone = False
                    self.Moves.append('lowJunc')
                allowed = 'Allowed'
            else:
                notAllowed = 'Not Allowed'
        for junctions in midPos:
            if (y,x,side) == junctions:
                if place == True and self.hasCone == True:
                    if side == 'L':
                        self.Time += 4
                    elif side == 'R':
                        self.Time += 4
                    self.Points += 4
                    self.hasCone = False
                    self.Moves.append('midJunc')
                allowed = 'Allowed'
            else:
                notAllowed = 'Not Allowed'
        for junctions in highPos:
            if (y,x,side) == junctions:
                if place == True and self.hasCone == True:
                    if side == 'L':
                        self.Time += 5
                    elif side == 'R':
                        self.Time += 5
                    self.Points += 5
                    self.hasCone = False
                    self.Moves.append('highJunc')
                allowed = 'Allowed'
            else:
                notAllowed = 'Not Allowed'
        if place == False:
            return notAllowed
    
    def Move(self,direction):
        y,x = self.position
        if direction == 'Up':
            y += 1
            self.Time += moveTime
            self.Moves.append('Up')
        elif direction == 'Down':
            y -= 1
            self.Time += moveTime
            self.Moves.append('Down')
        elif direction == 'Right':
            x += 1
            self.Time += moveTime
            self.Moves.append('Right')
        elif direction == 'Left':
            x -= 1
            self.Time += moveTime
            self.Moves.append('Left')
        self.position = (y,x)

    def getCone(self):
        y,x = self.position
        coneY,coneX = conePos
        y = abs(coneY-y)
        x = abs(coneX-x)
        self.Time += (x+y)*moveTime
        self.hasCone = True
        self.position = (coneY,coneX)

    
    def randomMove(self):
        y,x = self.position
        Actions = ['Up', 'Down', 'Left', 'Right', 'PlaceCone']
        allowedMoves = ['PlaceCone']
        #allowedMoves = []
        if (y+1 <= yCord[(len(yCord)-1)]):
            allowedMoves.append('Down')
        if (y-1 >= yCord[0]):
            allowedMoves.append('Up')
        if (x+1 <= xCord[(len(xCord)-1)]):
            allowedMoves.append('Right')
        if (x-1 >= xCord[0]):
            allowedMoves.append('Left')
        self.allowedMoves = allowedMoves

        randomAction = np.random.choice(self.allowedMoves)
        if (randomAction == 'PlaceCone' and self.hasCone == True):
            randomSide = np.random.choice(['L','R'])
            if self.compairJunction(y,x,randomSide) == 'Allowed':
                self.compairJunction(y,x,randomSide,True)
                self.getCone()
            elif self.compairJunction(y,x,randomSide) == 'Not Allowed' and randomSide == 'L':
                self.compairJunction(y,x,'R',True)
                self.getCone()
            else:
                self.compairJunction(y,x,'L',True)
                self.getCone()
        else:
            self.Move(randomAction)
        
    
    def isTimeUp(self):
        if self.Time >= 22:
            return True
        else:
            return False
    
    def getResults(self):
        return self.Points, self.Moves, self.Time
    
    def Test(self):
        print(self.Moves)

    def printMaze(self):
        for i in xCord:
            for v in yCord:
                if (v,i == self.position):
                    print('R', end=" ")
                else:
                    print('-', end=" ")

import numpy as np
import math
import time as tm
from Grid import Field
#from Brain import Brain

if __name__ == '__main__':
    field = Field()
    bestPoints = 0
    bestMoves = []
    bestTime = 0
    tic = tm.perf_counter()
    for i in range(10000000):
        if i % 1000000 == 0:
            print(i)
        
        #field.Test()
        while not field.isTimeUp():
            field.randomMove()
        points, moves, time = field.getResults()
        if points > bestPoints:
            bestPoints = points
            bestMoves.clear()
            bestMoves = moves
            bestTime = time
            #field.Test()
        field = Field()
    toc = tm.perf_counter()
print(bestMoves)
print(bestPoints)
print(bestTime)
print(f"Time {(toc - tic)/60:0.4f} minutes")

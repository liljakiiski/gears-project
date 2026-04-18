from GridSquare import GridSquare
import csv

def convertToCSV(gridKnowledge):

    map = []
    max_x = max(square.x for square in gridKnowledge if square.typo == 1)
    max_y = max(square.y for square in gridKnowledge if square.typo == 1)

    for i in range(max_y + 2):
        map.append([])
        for j in range(max_x + 2):
            square = next((sq for sq in gridKnowledge if sq.x == j and sq.y == i), None)
            if square:
                map[i].append(square.typo)
            else:
                map[i].append(0)
    
    map.reverse() # reverse the map to match the coordinate system

    with open('output.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        # writer.writerow(['Team: ', TEAM])
        writer.writerows(map)


gridKnowledge = [GridSquare(3, 0, 5), GridSquare(3, 1, 1), GridSquare(4, 1, 1), GridSquare(4, 2, 1), GridSquare(4, 3, 1), GridSquare(4, 4, 1), GridSquare(3, 4, 1), GridSquare(2, 4, 1), GridSquare(1, 4, 1), GridSquare(1, 3, 1), GridSquare(1, 2, 1), GridSquare(1, 1, 1), GridSquare(1, 0, 4), GridSquare(0, 1, 2),  GridSquare(2, 5, 3), GridSquare(2, 5, 3)]
convertToCSV(gridKnowledge)
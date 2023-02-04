import matplotlib.pyplot as plt
import sys

def plot_polygon(polygon):
    plt.rcParams["figure.autolayout"] = True
    x, y = zip(*polygon)
    plt.grid()
    plt.plot(x,y, marker='o')
    plt.show()

def read_polygon(filename):
    ''' input file format:
    Optimal Area Polygonization
    x1  y1
    x2  y2
     ...
    xn  yn
    .... '''
    polygon = list()
    with open(filename, 'r') as f: # open in read-only mode
        lines = f.readlines()
        for line in lines[1:]:
            if len(line.split()) != 2:
                break
            x = line.split()[0]
            y = line.split()[1]
            polygon.append((int(x), (int(y))))
    polygon.append(polygon[0]) # to "close" the polygon line
    return polygon

if __name__ == '__main__': 
    if len(sys.argv) != 3:
        print("Usage: ./python3 -i <polygon_file>")
        exit(0)
    filename = sys.argv[2]
    polygon = read_polygon(filename)
    plot_polygon(polygon)
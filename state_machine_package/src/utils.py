#!/usr/bin/env python

import codecs

class color:
   PURPLE = '\033[95m'
   CYAN = '\033[96m' #quando chiedo input
   DARKCYAN = '\033[36m'
   BLUE = '\033[94m'
   GREEN = '\033[92m' #ok
   YELLOW = '\033[93m' #info
   RED = '\033[91m' #errori
   BOLD = '\033[1m'
   UNDERLINE = '\033[4m'
   END = '\033[0m'

def load_txt(filename):
    # funzioncina che mi permette di aprire i file txt con codifica utf8
    # e salvare il contenuto in un vettore unico, dove ogni cella del
    # vettore corrisponde a un record
    with codecs.open(filename, "r", encoding="utf-8-sig") as file:
        row = file.read().split('\n')
        #[u'-200 -300 400 -90', u'-200 -300 400 -90', u'']
        if row[-1] == '':
            del row[-1]
        #[u'-200 -300 400 -90', u'-200 -300 400 -90']
        srow = [i.split(' ') for i in row]
        #print(srow[0])
        #[[u'-200', u'-300', u'400', u'-90'], [u'-200', u'-300', u'400', u'-90']]
    return srow

def write_txt(filename, data):
    QQ = []
    file = open(filename,'w')
    for i in range(0, len(data)): #riga
        for j in range(0, len(data[i])): #elemento all'interno
            if j == range(0, len(data[i]))[-1]: #se e' l'ultimo valore
                file.write(str(data[i][j]) + '\n')
            else:
                file.write(str(data[i][j]) + ' ')
        #q = [round(float(item),2) for item in data[i]]
        q = [item for item in data[i]]
        QQ.append(q)
    file.close()
    return QQ

def checkifduplicates(names, points):
    # to be called after the file has been loaded in memory
    new_points = []
    new_names = []
    for i in range(0,len(points)):
        # appends point only if the point is new compared to other points or if
        # the name is new compared to other point names. condition can be made
        # stronger if the name check is removed!
        if (points[i] not in new_points) or (names[i] not in new_names):
            new_points.append(points[i])
            new_names.append(names[i])
    return new_names, new_points

def load_points(points_list):
    points = []
    names = []
    for i in range(0,len(points_list)):
        names.append(str(points_list[i][0]))
        points.append([float(j) for j in points_list[i][1:]])
    #[[-200.0, -300.0, 400.0, -90.0], [-200.0, -300.0, 400.0, -90.0]]
    return names, points

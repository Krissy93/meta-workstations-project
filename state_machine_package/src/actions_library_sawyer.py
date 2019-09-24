#!/usr/bin/env python

import rospy
from utils import *
import rospkg

PKG_PATH = rospkg.RosPack().get_path('state_machine_package')

def action_list():
    """ Prints all the available actions present in this file. """

    rospy.loginfo(color.BOLD + color.PURPLE + '|-------------------|' + color.END)
    rospy.loginfo(color.BOLD + color.PURPLE + '| AVAILABLE ACTIONS |' + color.END)
    rospy.loginfo(color.BOLD + color.PURPLE + '| 1: MOVE TO POINT  |' + color.END)
    rospy.loginfo(color.BOLD + color.PURPLE + '| 2: OPEN GRIPPER   |' + color.END)
    rospy.loginfo(color.BOLD + color.PURPLE + '| 3: CLOSE GRIPPER  |' + color.END)
    rospy.loginfo(color.BOLD + color.PURPLE + '|-2: EXIT           |' + color.END)
    rospy.loginfo(color.BOLD + color.PURPLE + '|-------------------|' + color.END)

    actions = ['MOVE TO POINT', 'OPEN GRIPPER', 'CLOSE GRIPPER']

    return actions

def fill_angles(angles_list):
    ''' La lista di valori fornita e' gia' in ordine di giunto,
    ad esempio [0.0, -1.2, 3.4, -0.5, 1.5, -2.3, 0.0] corrispondono a j0 -> j6 '''

    angles = {'right_j6': float(angles_list[6]), 'right_j5': float(angles_list[5]),
              'right_j4': float(angles_list[4]), 'right_j3': float(angles_list[3]),
              'right_j2': float(angles_list[2]), 'right_j1': float(angles_list[1]),
              'right_j0': float(angles_list[0])}

    return angles

def angles2list(angles):
    ''' Partendo da una posizione del robot presa con limb.joint_angles(),
    riempie una lista di punti nel formato di salvataggio. '''

    angles_list = []

    angles_list.append(angles['right_j0'])
    angles_list.append(angles['right_j1'])
    angles_list.append(angles['right_j2'])
    angles_list.append(angles['right_j3'])
    angles_list.append(angles['right_j4'])
    angles_list.append(angles['right_j5'])
    angles_list.append(angles['right_j6'])

    return angles_list

def go2trajectory(filename):
    ''' Puo' prendere in ingresso sia la lista gia' fatta di pti,
    sia il file gia' corretto per il caricamento dei punti nel sistema.
    I punti vanno messi in una lista nella quale sono inseriti secondo la modalita'
    dizionario-joints che si vede in zero_angles. '''
    # sets speed limit
    VEL = 0.3

    limb = intera_interface.Limb('right')
    limb.set_joint_position_speed(VEL)

    # opens file
    file = load_txt(filename)
    # file e' fatto cosi':
    # [[u'0.0', u'1.0', u'2.0', u'3.0', u'4.0', u'5.0', u'6.0'],
    # [u'0.1', u'1.1', u'2.1', u'3.1', u'4.1', u'5.1', u'6.1'],
    # [u'0.2', u'1.2', u'2.2', u'3.2', u'4.2', u'5.2', u'6.2']]
    # quindi file[0] accede alla prima riga del file, file[0][0] printa u'0.0'.
    # vanno tutti convertiti in float con un cast: float(file[punto][giunto])
    npti = len(file)

    # crea una lista di dizionari contenenti i valori dei giunti
    # lunga tanto quanto i punti
    for i in range(0, npti):
        # file[i] corrisponde alla riga di punti
        # chiamando fill_angles con la riga file[i] ritorno un dizionario
        # che poi appendo alla lista come debug:
        # list_point.append(fill_angles(file[i])
        # muovo il robot con la sua funzione interna chiamando
        # la corrispondente riga di punti
        angles = fill_angles(file[i])
        rospy.loginfo(color.BOLD + color.YELLOW + '-- SAWYER SI SPOSTA IN POSIZIONE ' +  str(i+1) + '! --' + color.END)
        limb.move_to_joint_positions(angles)
        # per assicurare che il robot vada in posizione prima di mandargli il secondo messaggio
        rospy.sleep(0.5)
    rospy.loginfo(color.BOLD + color.GREEN + '-- MOVIMENTO CONCLUSO! --' + color.END)

def point2file(filename, punto):
    ''' Scrive in blocco il punto preso da sawyer in un file il cui nome e' scelto
    in automatico in base a quelli gia' presenti nella sottocartella e passato al metodo. '''

    # trasforma il punto in lista
    angles_list = angles2list(punto)

    file = open(filename,'a')
    for i in range(0, len(angles_list)):
        file.write(str(angles_list[i]))

        # se e' alla fine scrive a capo
        if i == len(angles_list):
            file.write('\n')
        else:
            # altrimenti mette lo spazio
            file.write(' ')

    file.close()

def get_files(pkg_path):
    ''' Loads all the files in the given folder and returns the list of files
    with their corresponding path. By doing this, the user can select each file using the
    filename as handler and execute it '''

    # creates a list of file paths contained in the operations folder of the package.
    files = filter(os.path.isfile, glob.glob(pkg_path + '/traiettorie/' + "*.txt"))
    # orders them according to modification date. Last modifified items are at the bottom of the list
    files.sort(key=lambda x: os.path.getmtime(x))

    return files

def hello():
    ''' Metodo che esegue la traiettoria del saluto di Sawyer '''

    # creates the limb instance
    limb = intera_interface.Limb('right')
    # sets them to zero and moves the robot to the zero position
    rospy.loginfo(color.BOLD + color.YELLOW + '-- SAWYER SI PREPARA... --' + color.END)
    angles = fill_angles([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    limb.move_to_joint_positions(angles)

    # store the first wave position
    wave_1 = {'right_j6': -1.5126, 'right_j5': -0.3438, 'right_j4': 1.5126,
              'right_j3': -1.3833, 'right_j2': 0.03726, 'right_j1': 0.3526, 'right_j0': -0.4259}
    # store the second wave position
    wave_2 = {'right_j6': -1.5101, 'right_j5': -0.3806, 'right_j4': 1.5103,
              'right_j3': -1.4038, 'right_j2': -0.2609, 'right_j1': 0.3940, 'right_j0': -0.4281}

    # wave three times
    rospy.loginfo(color.BOLD + color.YELLOW + '-- SAWYER DICE CIAO! --' + color.END)

    for _move in range(3):
         limb.move_to_joint_positions(wave_1)
         rospy.sleep(0.5)
         limb.move_to_joint_positions(wave_2)
         rospy.sleep(0.5)

    rospy.loginfo(color.BOLD + color.GREEN + '-- MOVIMENTO CONCLUSO! --' + color.END)

def cerchio():
    ''' Metodo che esegue la traiettoria del saluto di Sawyer '''

    # creates the limb instance
    limb = intera_interface.Limb('right')
    # sets them to zero and moves the robot to the zero position
    rospy.loginfo(color.BOLD + color.YELLOW + '-- SAWYER SI PREPARA... --' + color.END)
    angles = fill_angles([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    limb.move_to_joint_positions(angles)

    # store the first wave position
    wave_1 = {'right_j6': -1.5126, 'right_j5': -0.3438, 'right_j4': 1.5126,
              'right_j3': -1.3833, 'right_j2': 0.03726, 'right_j1': 0.3526, 'right_j0': -0.4259}
    # store the second wave position
    wave_2 = {'right_j6': -1.5101, 'right_j5': -0.3806, 'right_j4': 1.5103,
              'right_j3': -1.4038, 'right_j2': -0.2609, 'right_j1': 0.3940, 'right_j0': -0.4281}

    # wave three times
    rospy.loginfo(color.BOLD + color.YELLOW + '-- SAWYER DICE CIAO! --' + color.END)

    for _move in range(3):
         limb.move_to_joint_positions(wave_1)
         rospy.sleep(0.5)
         limb.move_to_joint_positions(wave_2)
         rospy.sleep(0.5)

    rospy.loginfo(color.BOLD + color.GREEN + '-- MOVIMENTO CONCLUSO! --' + color.END)

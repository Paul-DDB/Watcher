import numpy as np
import cv2 as cv
import datetime
import os
import matplotlib.pyplot as plt
import serial
import serial.tools.list_ports
import time

# Fonctions nécessaires au traitement en temps réel
ecart = 0.1
deg = 2
sizex = 500
sizey = 500
barre_goal = 500
taille = [i*ecart for i in range(int(sizex/ecart))]

instant_initial = 0.0

dernier_mouvement = 0.0
ecart_mouvements = 0.5

def fonction_transfert(x):
    """
        float -> float
        Renvoie le résultat du calcul fait à partir de la valeur en entrée. Utilisée pour le
        passage de la coordonnée observée à l'angle voulu pour le moteur.
    """
    return -x

def afficher_graph(abs, ord):
    """
        list x list -> None
        Trace l'ord en fonction de l'abs en utilisant pyplot.
    """

    plt.clf()
    plt.plot(abs, ord, linewidth=1, color = 'red')
    plt.show()

    return None


def approx_poly(x, y, degre, bornes, ecart_echant):
    """
        list x list x int x tuple x float -> list x list
        Réalise une approximation polynomiale de y en fonction de x
        à un degré 'degre' sur le segment bornes, et renvoie les coefficients de cette
        approximation.
    """

    x_pred = [bornes[0]]
    while x_pred[-1] < bornes[1]:
        x_pred.append(x_pred[-1] + ecart_echant)

    coef = np.polyfit(x, y, degre) #On calcule les coefficients
    fit = np.poly1d(coef) #On crée le modèle correspondant aux coefficients calculés
    y_pred = fit(x_pred)

    return(x_pred, y_pred)


# On récupère la date pour les logs
now = datetime.datetime.now()
now = str(now.year) + '_' + str(now.month) + '_' + str(now.day) + '_' + str(now.hour) + '.' + str(now.minute) + '.' + str(now.second)
video_source = 0

# Décommenter la première ligne pour traiter une vidéo préenregistrée, la deuxième pour la caméra live
cap = cv.VideoCapture(video_source)

# Différentes informations sur la capture vidéo
print("FRAME_WIDTH :",cap.get(cv.CAP_PROP_FRAME_WIDTH ))
print("FRAME_HEIGHT :",cap.get(cv.CAP_PROP_FRAME_HEIGHT ))
print("FRAME_COUNT :",cap.get(cv.CAP_PROP_FRAME_COUNT))
print("FPS :",cap.get(cv.CAP_PROP_FPS))

# Définition de la bande de couleur à isoler
middle = 25
span = 4

lower_yellow = np.array([middle-span, 150, 130])
upper_yellow = np.array([middle+span, 255, 255])

middle2 = 35

lower_merlin = np.array([middle2-span, 63, 120])
upper_merlin = np.array([middle2+span, 255, 255])

middle3 = 45

lower_vert = np.array([middle3+span, 201, 134])
upper_vert = np.array([80, 255, 255])

lower_vert_hsv = np.array([56, 82, 182])
upper_vert_hsv = np.array([84, 227, 255])

lower_actuel = lower_vert_hsv
upper_actuel = upper_vert_hsv

data = [[], [], []]
data_approx = [[], []]

# Définition de la matrice d'ouverture morphologique (réduction du bruit sur l'image)
kernel = np.ones((5,5), np.uint8)

script_dir = os.path.dirname(__file__)
rel_path = r'logs//' + now + ".txt"
abs_file_path = os.path.join(script_dir, rel_path)
file = open(abs_file_path, 'x', encoding = 'utf-8')

file.write("--- Début du programme" + '\n')

ports = serial.tools.list_ports.comports(include_links=False)

if (len(ports) != 0): # on a trouvé au moins un port actif

    if (len(ports) > 1):     # affichage du nombre de ports trouvés
        file.write("--- " + str(len(ports)) + " ports actifs ont été trouvés" + '\n')
        print (str(len(ports)) + " ports actifs ont ete trouves:") 
    else:
        file.write("--- 1 port actif a été trouvé" + '\n')
        print ("1 port actif a ete trouve:")

    ligne = 1

    for port in ports :  # affichage du nom de chaque port
        print(str(ligne) + ' : ' + port.device)
        ligne = ligne + 1

    portChoisi = input('Écrivez le numéro du port désiré:')

    print('1: 9600   2: 38400    3: 115200')

    baud = input('Écrivez le baud rate désiré (9600 grandement conseillé) :')
    baud = int(baud)
    if (baud == 1):
        baud = 9600
    if (baud == 2):
        baud = 38400
    if (baud == 3):
        baud = 115200
    portChoisi = int(portChoisi)
    # on établit la communication série
    arduino = serial.Serial(ports[portChoisi - 1].device, baud, timeout=1)
    
    print('Connexion à ' + arduino.name + ' à un baud rate de ' + str(baud))
    file.write('--- Connexion à ' + arduino.name + ' à un baud rate de ' + str(baud) + '\n')
        
else: # on n'a pas trouvé de port actif
    print("Aucun port actif n'a été trouvé")
    file.write("Aucun port actif n'a été trouvé" + '\n')

instant_initial = time.time()

while cap.isOpened():
    # On prend une frame de la capture
    ret, frame = cap.read()

    # Si erreur on stop tout
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # On convertir l'image au format HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # On isole seulement la bande de couleur définie plus haut
    mask = cv.inRange(hsv, lower_actuel, upper_actuel)

    # On applique une ouverture morphologique avec la matrice définie plus haut pour réduire le bruit de l'image isolé
    opening = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

    # On récupère une liste des contours de l'image (forme fermé)
    contours, hierarchy = cv.findContours(opening, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    if len(contours)!=0:
        cnt = contours[0]

        # On approxime le contour à un cercle dont on récupère le centre et le rayon
        (Cx,Cy),radius = cv.minEnclosingCircle(cnt)
        data[0].append(Cx)
        data[1].append(470-Cy)
        data[2].append(time.time() - instant_initial)

        data_approx[0].append(Cx)
        data_approx[1].append(470-Cy)

        center = (int(Cx),int(Cy))
        radius = int(radius)
        file.write(str(data[0][-1]) + ";" + str(data[1][-1]) + '\n')
        # On retrace la balle sur l'image de sortie (overlay)
        cv.circle(frame,center,radius,(0,255,0),1)
        cv.drawMarker(frame, center, (0,255,0), cv.MARKER_CROSS, 10000, 1)
        """
        if len(data[0]) > 5:
            plt.clf()
        plt.axis([0, sizex*1.1, 0, sizey*1.1])
        plt.scatter(Cx, 470-Cy, c = 'blue')
        coef = np.polyfit(data_approx[0], data_approx[1], deg)
        fit = np.poly1d(coef)
        approx = plt.plot(taille, fit(taille), c = 'black')
        plt.plot([barre_goal for i in range(sizey)], [i for i in range(sizey)])
        plt.scatter(barre_goal, fit(barre_goal), c = 'red')
        plt.pause(0.00000001)
        """
        if time.time() - dernier_mouvement > ecart_mouvements:
            message = str(fonction_transfert(Cx))
            arduino.write(message.encode())
            dernier_mouvement = time.time()
            print(message)
            print(arduino.readline())
        

#    elif len(data_approx[0]) != 0:
#        data_approx[0].append(data_approx[0][-1])
#        data_approx[1].append(data_approx[1][-1])


    # On redimensionne les images pour avoir une fenêtre de taille convenable
    sizeList = [(1920,1080),(1280,720),(960, 540)]
    size = sizeList[1]
    frame = cv.resize(frame, size)
    mask = cv.resize(mask, size)
    opening = cv.resize(opening, size)

    # On affiche les images
    cv.imshow('frame', frame)
    cv.imshow('mask', mask)
    cv.imshow('opening', opening)


    if cv.waitKey(1) == ord('q'):
        file.write("--- Demande d'arrêt" + '\n')
        plt.close()
        break

# On termine le programme en fermant les fenêtres et en éteignant la caméra
file.write("--- Fin de la prise d'image" + '\n')
cap.release()
file.write("--- Fermeture des pages" + '\n')
cv.destroyAllWindows()
file.write("--- Fermeture du canal de discussion Arduino" + '\n')
arduino.close()


print("")
print("")
print("")
print("Tracer les informations obtenues pendant la prise d'image ?")
print("1 - OUI")
print("2 - NON")
print("")
choix = int(input(">   "))

if choix == 1:
    print("")
    print("Quel graphe tracer ?")
    print("1 - y en fonction de x")
    print("2 - y en fonction du temps")
    print("3 - x en fonction du temps")
    print("4 - Exit")
    print("")
    choix = int(input(">   "))
    if choix == 1:
        plt.plot(data[0], data[1])
        plt.plot([barre_goal for i in range(sizey)], [i for i in range(sizey)])
        plt.show()
    elif choix == 2:
        plt.plot(data[2], data[1])
        plt.show()
    elif choix == 3:
        plt.plot(data[2], data[0])
        plt.show()

file.write("--- Fin de la lecture des données")

file.write("--- Fermeture fichier")
file.close()



#L = [i for i in range(len(data[0]))]
#plt.plot(data[0], data[1])
#plt.show()
#plt.close()
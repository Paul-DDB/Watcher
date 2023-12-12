import numpy as np
import cv2 as cv
import datetime
import os
import matplotlib.pyplot as plt
import serial
import serial.tools.list_ports
import time

#py -m pip --proxy ridf@10.0.0.1:3128 install <nom du module>

# Fonctions nécessaires au traitement en temps réel
ecart = 0.1
deg = 2
sizex = 500
sizey = 500
barre_goal = 500
taille = [i*ecart for i in range(int(sizex/ecart))]

radius_j = 0
derniere_pos_j_x = 0
derniere_pos_j_y = 0

instant_initial = 0.0

dernier_mouvement = 0.0
ecart_mouvements = 0.5

def fonction_transfert(x,y):
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

def returnCameraIndexes():
    """
        None -> list
        Renvoie la liste des indices de caméras disponibles sur le PC.
        Ne pas s'inquiéter des erreurs !
    """
    # checks the first 10 indexes.
    index = 0
    arr = []
    i = 10
    while i > 0:
        cap = cv.VideoCapture(index)
        if cap.read()[0]:
            arr.append(index)
            cap.release()
        index += 1
        i -= 1
    return arr



# On récupère la date pour les logs
now = datetime.datetime.now()
now = str(now.year) + '_' + str(now.month) + '_' + str(now.day) + '_' + str(now.hour) + '.' + str(now.minute) + '.' + str(now.second)

# On récupère le chemin vers le fichier, et vers les  logs
script_dir = os.path.dirname(__file__)
rel_path = r'logs//' + now + ".txt"
abs_file_path = os.path.join(script_dir, rel_path)
abs_folder_path = os.path.join(script_dir, r'logs//')

if not os.path.exists(abs_folder_path): # Si 'logs' n'existe pas, on le crée
    os.makedirs(abs_folder_path)
    
file = open(abs_file_path, 'x', encoding = 'utf-8')

file.write("--- Début du programme" + '\n')
print()
print("Quelle source utiliser ? (c ou v)")
choix_source = input("> ")

if choix_source == 'c':
    cameras = returnCameraIndexes()
    if len(cameras) == 0:
        print("Aucune caméra trouvée")
        file.write("--- Aucune caméra trouvée, fin du programme" + '\n')
        exit()
        
    print()
    print("Choisir une caméra parmi :")
    compteur = 0
    for elt in cameras:
        print(str(compteur) + ' : ' + str(elt))
        

    video_source = int(input("> "))
    file.write("--- Source choisie : caméra " + str(video_source) + '\n')
    
elif choix_source == 'v':
    print()
    print("Quelle vidéo lire ?")
    video_source = input("> ")
    file.write("--- Source choisie : vidéo " + video_source)

else:
    print("ERREUR : source invalide")
    file.write("--- Source choisie invalide, fin du programme" + '\n')
    exit()
    
    
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

# /!\ La bonne /!\
lower_vert_hsv = np.array([56, 82, 182]) 
upper_vert_hsv = np.array([84, 227, 255])

lower_actuel = lower_vert_hsv
upper_actuel = upper_vert_hsv

# Celle pour le joueur
lower_noir = np.array([])
upper_noir = np.array([])

# On crée les listes pour plus tard
data = [[], [], []]
data_approx = [[], []]
pos_joueur = []

# Définition de la matrice d'ouverture morphologique (réduction du bruit sur l'image)
kernel = np.ones((5,5), np.uint8)

print()
ports = serial.tools.list_ports.comports(include_links=False)

if (len(ports) != 0): # on a trouvé au moins un port actif

    if (len(ports) > 1):     # affichage du nombre de ports trouvés
        file.write("--- " + str(len(ports)) + " ports actifs ont été trouvés" + '\n')
        print (str(len(ports)) + " ports actifs ont été trouvés:") 
    else:
        file.write("--- 1 port actif a été trouvé" + '\n')
        print ("1 port actif a ete trouve:")

    ligne = 1
    for port in ports :  # affichage du nom de chaque port
        print(str(ligne) + ' : ' + port.device)
        ligne = ligne + 1
    print("Écrivez le numéro du port désiré :")
    portChoisi = input('> ')
    print()
    print('1: 9600   2: 38400    3: 115200')
    print("Écrivez le baud rate désiré (9600 grandement conseillé) :")
    baud = input('> ')
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
        
else: # Si on n'a pas trouvé de port actif
    print("Aucun port actif n'a été trouvé")
    file.write("Aucun port actif n'a été trouvé" + '\n')

print()
print("Tracer en temps réel la trajectoire ?")
print("1 : OUI")
print("2 : NON")
choix_trace = 2 - int(input("> "))

print()
print("Envoyer les instructions à la carte Arduino ?")
print("1 : OUI")
print("2 : NON")
choix_arduino = 2 - int(input("> "))

choix_fait = 0
if choix_arduino:
    while not choix_fait:
        print()
        print("Quelle grandeur envoyer à la carte ?")
        print("0 : ANNULER LA COMMUNICATION")
        print("1 : La position en abscisse")
        print("2 : La position en ordonnée")
        print("3 : L'ordonnée prévue au niveau du gardien")
        print("4 : La position calculée par la fonction de transfert")
        choix_sortie = int(input("> "))
        if choix_sortie in [1,2,3,4]:
            choix_fait = 1
        elif choix_sortie == 0:
            choix_arduino = 0

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
    mask_noir = cv.inRange(hsv, lower_noir, upper_noir)

    # On applique une ouverture morphologique avec la matrice définie plus haut pour réduire le bruit de l'image isolé
    opening = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    opening_noir = cv.morphologyEx(mask_noir, cv.MORPH_OPEN, kernel)

    # On récupère une liste des contours de l'image (forme fermé)
    contours, hierarchy = cv.findContours(opening, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    contours_j, hierarchy_j = cv.findContours(opening_noir, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    if len(contours)!=0:
        cnt = contours[0]

        # On approxime le contour à un cercle dont on récupère le centre et le rayon
        (Cx,Cy),radius = cv.minEnclosingCircle(cnt)
        data[0].append(Cx)
        data[1].append(470-Cy)
        data[2].append(time.time() - instant_initial)
        
        if len(contours_j) !=0:
            #Si on repère du noir, on actualise la position du joueur
            cnt_j = contours_j[0]
            (Cx_j,Cy_j),radius_j = cv.minEnclosingCircle(cnt_j)
            derniere_pos_j_y = Cy_j
            derniere_pos_j_x = Cx_j
            
        center_j = (int(derniere_pos_j_x), derniere_pos_j_y)
        radius_j = int(radius_j)
        
        #On trace une cercle rouge sur la dernière position connue du joeur
        cv.circle(frame,center_j,radius_j,(255,0,0),1)
        cv.drawMarker(frame, center_j, (0,0,255), cv.MARKER_CROSS, 10000, 1)
        
        pos_joueur.append(470 - derniere_pos_j)

        data_approx[0].append(Cx)
        data_approx[1].append(470 - Cy)

        center = (int(Cx),int(Cy))
        radius = int(radius)
        file.write(str(data[0][-1]) + ";" + str(data[1][-1]) + ";" + str(pos_joueur[-1]) + '\n')
        #On retrace la balle sur l'image de sortie (overlay)
        cv.circle(frame,center,radius,(0,255,0),1)
        cv.drawMarker(frame, center, (0,255,0), cv.MARKER_CROSS, 10000, 1)
        
        if choix_trace: # Si on a décidé de tracer les données en temps réel
            if len(data[0]) > 5: # On limite le nombre de valeurs tracées
                plt.clf()
            plt.axis([0, sizex*1.1, 0, sizey*1.1])
            plt.scatter(Cx, 470-Cy, c = 'blue')
            coef = np.polyfit(data_approx[0], data_approx[1], deg)
            fit = np.poly1d(coef)
            approx = plt.plot(taille, fit(taille), c = 'black') # On trace en noir la trajectoire prévue
            plt.plot([barre_goal for i in range(sizey)], [i for i in range(sizey)])
            plt.scatter(barre_goal, fit(barre_goal), c = 'red') 
            # On trace en rouge le point  prévud'impact sur la barre
            plt.pause(0.00000001)
        
        
        if (time.time() - dernier_mouvement > ecart_mouvements) and choix_arduino:
            #On n'envoie les données à la carte Arduino que si l'on l'a demandé ET pas en permanence
            coef = np.polyfit(data_approx[0], data_approx[1], deg)
            fit = np.poly1d(coef)
            if choix_sortie == 1:
                message = str(Cx)
            elif choix_sortie == 2:
                message = str(Cy)
            elif choix_sortie == 3:
                message = str(fit(barre_goal))
            elif choix_sortie == 4:
                message = str(fonction_transfert(fit(barre_goal)), derniere_pos_j)
            arduino.write(message.encode())
            dernier_mouvement = time.time()

    elif len(data_approx[0]) != 0 and choix_trace:
        data_approx[0].append(data_approx[0][-1])
        data_approx[1].append(data_approx[1][-1])


    # On redimensionne les images pour avoir une fenêtre de taille convenable
    sizeList = [(1920,1080),(1280,720),(960, 540)]
    size = sizeList[1]
    frame = cv.resize(frame, size)
    mask = cv.resize(mask, size)
    opening = cv.resize(opening, size)

    # On affiche les images
    cv.imshow('frame', frame)
    
    # Enlever les """ pour afficher les 3 fenêtres avec le masque
    """
    cv.imshow('mask', mask)
    cv.imshow('opening', opening)
    cv.imshow('mask_noir', mask_noir)
    cv.imshow('opening_noir', opening_noir)
    """


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

if choix_arduino:
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
    # On gère le cas où l'utilisateur veut tracer des info obtenues
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

file.write("--- Fin de la lecture des données" + '\n')

file.write("--- Fermeture fichier" + '\n')
file.close()

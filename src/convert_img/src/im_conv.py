#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os
from std_msgs.msg import Int32MultiArray, MultiArrayDimension


class ImageConverter:
    def __init__(self):
        # Initialisation de ROS
        rospy.init_node('im_conv', anonymous=True)
        
        # Publisher pour le tableau binaire
        self.map_publisher = rospy.Publisher('binary_map_topic', Int32MultiArray, queue_size=10)
        
        # Publisher pour la position du robot (initiale et 
        # /pos_robot

        # Paramètres du noeud
        self.map_file = rospy.get_param('~map_file', '/home/projet_rob_mobile/map.pgm')  # Fichier .pgm
        self.output_dir = rospy.get_param('~output_dir', '/home/projet_rob_mobile/')  # Dossier de sortie
        self.scale_factor = rospy.get_param('~scale_factor', 2)  # Facteur d'agrandissement
        self.grid_step = rospy.get_param('~grid_step', 5)  # Pas de la grille en pixels

        rospy.loginfo(f"Processing map file: {self.map_file}")

        # Vérifier si le fichier existe
        if not os.path.exists(self.map_file):
            rospy.logerr(f"Map file {self.map_file} not found.")
            raise FileNotFoundError(f"Map file {self.map_file} not found.")

        # Vérifier si le répertoire de sortie existe
        if not os.path.exists(self.output_dir):
            rospy.loginfo(f"Output directory {self.output_dir} does not exist. Creating it.")
            os.makedirs(self.output_dir)

        self.process_map()

    def process_map(self):
        try:
            # Charger l'image .pgm
            map_image = cv2.imread(self.map_file, cv2.IMREAD_UNCHANGED)
            if map_image is None:
                rospy.logerr("Failed to load the map image.")
                return

            rospy.loginfo(f"Map image loaded: {map_image.shape}")

            # Identifier et rogner la zone avec le maximum de pixels noirs
            cropped_image = self.crop_to_black_area(map_image)

            # Grossir l'image
            scaled_image = cv2.resize(
                cropped_image,
                None,
                fx=self.scale_factor,
                fy=self.scale_factor,
                interpolation=cv2.INTER_NEAREST
            )

            # Appliquer le traitement des pixels noirs
            processed_image = self.expand_black_pixels(scaled_image)

            # Générer une grille adaptée aux obstacles
            no_grid_image, grid_image = self.add_obstacle_grid(processed_image)
            
            # Sauvegarder l'image sans la grille
            no_grid_output_file = os.path.join(self.output_dir, 'map_without_grid.png')
            cv2.imwrite(no_grid_output_file, no_grid_image)
            rospy.loginfo(f"Image without grid saved to {no_grid_output_file}")
            
            # Sauvegarder l'image avec la grille
            output_file = os.path.join(self.output_dir, 'map_with_obstacle_grid.png')
            cv2.imwrite(output_file, grid_image)
            rospy.loginfo(f"Image processed and saved to {output_file}")
            
            # Générer un tableau binaire à partir de l'image traitée
            # Pixels blancs (obstacles) -> 1, Pixels rouges (zones libres) -> 0
            grid_image_gray = cv2.cvtColor(no_grid_image, cv2.COLOR_BGR2GRAY)
            binary_map = np.where(grid_image_gray == 255, 1, 0)
            rows, cols = binary_map.shape
            print(f"Number of rows: {rows}, Number of columns: {cols}")
            
            # Publier le tableau binaire
            self.publish_binary_map(binary_map)
            
            # Convertir le tableau NumPy en tableau Python
            binary_map_python = binary_map.tolist()
            
            # Afficher une partie du tableau pour vérification
            #print(binary_map_python)  # Affiche les 5 premières lignes

            # Sauvegarder le tableau binaire dans un fichier texte
            binary_output_file = os.path.join(self.output_dir, 'obstacle_map.txt')
            np.savetxt(binary_output_file, binary_map, fmt='%d', delimiter='')
            rospy.loginfo(f"Obstacle map saved to {binary_output_file}")

            
            #obstacle_map = np.load('/home/projet_rob_mobile/obstacle_map.txt')
            #print(obstacle_map)
            

        except Exception as e:
            rospy.logerr(f"Failed to process map: {e}")

    def crop_to_black_area(self, image):
        """
        Identifie et rogner la zone contenant le maximum de pixels noirs dans l'image.
        """
        # Identifier les pixels noirs
        black_pixels = np.where(image == 0)

        if black_pixels[0].size == 0 or black_pixels[1].size == 0:
            rospy.logwarn("No black pixels detected in the image.")
            return image  # Retourne l'image originale si aucun pixel noir n'est trouvé

        # Calculer les limites du rectangle englobant
        min_row, max_row = np.min(black_pixels[0]), np.max(black_pixels[0])
        min_col, max_col = np.min(black_pixels[1]), np.max(black_pixels[1])

        rospy.loginfo(f"Cropping to rectangle: ({min_row}, {min_col}) - ({max_row}, {max_col})")

        # Rogner l'image
        cropped_image = image[min_row:max_row + 1, min_col:max_col + 1]
        return cropped_image

    def expand_black_pixels(self, image):
        """
        Étend les pixels noirs en fonction de leurs voisins pour combler les espaces proches.
        """
        # Réduction du bruit
        denoised_image = cv2.medianBlur(image, 3)

        # Identification des pixels noirs
        _, binary_image = cv2.threshold(denoised_image, 50, 255, cv2.THRESH_BINARY_INV)

        # Dilatation des pixels noirs
        dilation_size = 15 # 20
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (2 * dilation_size + 1, 2 * dilation_size + 1),
            (dilation_size, dilation_size)
        )
        dilated_image = cv2.dilate(binary_image, kernel)

        return dilated_image

    def add_obstacle_grid(self, image):
        """
        Ajoute une grille qui s'adapte aux obstacles en les englobant dans des cellules inatteignables.
        """
        grid_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)  # Convertir en couleur pour dessiner les grilles
        no_grid_image = grid_image.copy()
        step = self.grid_step

        # Définir un seuil pour considérer une cellule comme un obstacle
        obstacle_threshold = 0.1  # 10% des pixels doivent être noirs

        # Parcourir l'image avec des cellules de taille `step x step`
        obstacle_cells = 0
        for y in range(0, image.shape[0], step):
            for x in range(0, image.shape[1], step):
                # Définir la région d'intérêt (ROI)
                roi = image[y:y + step, x:x + step]

                # Vérifier si la cellule contient des pixels noirs (obstacles)
                if np.sum(roi == 0) / roi.size > obstacle_threshold:
                    obstacle_cells += 1
                    # Colorer la cellule pour indiquer qu'elle est inatteignable
                    cv2.rectangle(
                        grid_image,
                        (x, y),
                        (x + step, y + step),
                        (0, 0, 255),  # Rouge
                        -1  # Remplir la cellule
                    )

                # Tracer les bordures de la cellule
                cv2.rectangle(
                    grid_image,
                    (x, y),
                    (x + step, y + step),
                    (255, 255, 255),  # Blanc
                    1  # Bordure fine
                )

        rospy.loginfo(f"Number of obstacle cells: {obstacle_cells}")
        return no_grid_image, grid_image
        
    def publish_binary_map(self, binary_map):
	    """
	    Publie le tableau binaire (converti en tableau Python) sur un topic ROS.
	    """
	    msg = Int32MultiArray()

	    # Ajouter les dimensions
	    rows, cols = binary_map.shape
	    msg.layout.dim.append(MultiArrayDimension(label="rows", size=rows, stride=rows * cols))
	    msg.layout.dim.append(MultiArrayDimension(label="cols", size=cols, stride=cols))

	    # Msg.data
	    msg.data = binary_map.tolist()

	    # Publier le message
	    self.map_publisher.publish(msg)
	    rospy.loginfo(f"Published binary map of size {rows}x{cols} on 'binary_map_topic'")



if __name__ == '__main__':
    try:
        ImageConverter()
    except rospy.ROSInterruptException:
        rospy.loginfo("Image converter node terminated.")


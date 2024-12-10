#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os


class ImageConverter:
    def __init__(self):
        # Initialisation de ROS
        rospy.init_node('im_conv', anonymous=True)

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

            # Appliquer une binarisation adaptative
            binarized_image = cv2.adaptiveThreshold(
                scaled_image,
                255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV,
                11,  # Taille du voisinage pour calculer le seuil
                2    # Constante soustraite
            )

            # Générer une grille adaptée aux obstacles
            grid_image = self.add_obstacle_grid(binarized_image)

            # Sauvegarder l'image avec la grille
            output_file = os.path.join(self.output_dir, 'map_with_obstacle_grid.png')
            cv2.imwrite(output_file, grid_image)
            rospy.loginfo(f"Image processed and saved to {output_file}")

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

    def add_obstacle_grid(self, image):
        """
        Ajoute une grille qui s'adapte aux obstacles en les englobant dans des cellules inatteignables.
        """
        grid_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)  # Convertir en couleur pour dessiner les grilles
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
        return grid_image


if __name__ == '__main__':
    try:
        ImageConverter()
    except rospy.ROSInterruptException:
        rospy.loginfo("Image converter node terminated.")


#!/usr/bin/env python3

import rospy
import tf
import cv2
import numpy as np
import os
import termios
import sys
import tty
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Int32MultiArray, MultiArrayDimension, Float32MultiArray, Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray

class ImageConverter:
    def __init__(self):
        # Initialisation de ROS
        rospy.init_node('im_conv', anonymous=True)

        # Publisher pour le tableau binaire
        self.map_publisher = rospy.Publisher('binary_map_topic', Int32MultiArray, queue_size=10)
        
        # Publisher pour la position du robot
        self.pos_robot = rospy.Publisher('pos_robot', Float32MultiArray, queue_size=10)
        self.clicked_point_pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=10)

        # Publisher pour le chemin
        self.path_publisher = rospy.Publisher('path', Path, queue_size=10)

        # Publisher pour les coins de la carte
        self.corners_publisher = rospy.Publisher('map_corners', Float64MultiArray, queue_size=10)

        # Paramètres du noeud
        self.output_dir = rospy.get_param('~output_dir', '/home/projet_rob_mobile/')  # Dossier de sortie
        self.scale_factor = rospy.get_param('~scale_factor', 2)  # Facteur d'agrandissement
        self.grid_step = rospy.get_param('~grid_step', 5)  # Pas de la grille en pixels
        
        # Liste pour enregistrer les trajectoires
        self.trajectory = []

        rospy.loginfo("Waiting for user input to fetch map...")

        # Vérifier si le répertoire de sortie existe
        if not os.path.exists(self.output_dir):
            rospy.loginfo(f"Output directory {self.output_dir} does not exist. Creating it.")
            os.makedirs(self.output_dir)

        self.run()

    def get_key(self):
        """Capture une touche entrée dans le terminal."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def get_map_dimensions(self):
        """
        Récupère les dimensions de la carte rognée en pixels et en mètres.
        """
        rospy.wait_for_service('/dynamic_map')
        try:
            # Appeler le service
            map_service = rospy.ServiceProxy('/dynamic_map', GetMap)
            response = map_service()

            # Extraire les informations de la carte
            resolution = response.map.info.resolution
            width = response.map.info.width
            height = response.map.info.height

            # Convertir la carte en format OpenCV
            map_data = np.array(response.map.data, dtype=np.int8)
            map_image = map_data.reshape((height, width))

            # Rogner la carte sur les zones contenant des pixels noirs
            cropped_image = self.crop_to_black_area(map_image)

            # Calculer les dimensions de la carte rognée en pixels
            cropped_height, cropped_width = cropped_image.shape

            # Calculer les dimensions de la carte rognée en mètres
            cropped_width_in_meters = cropped_width * resolution
            cropped_height_in_meters = cropped_height * resolution

            rospy.loginfo(f"Cropped map dimensions in pixels: {cropped_width} x {cropped_height}")
            rospy.loginfo(f"Cropped map dimensions in meters: {cropped_width_in_meters:.2f} m x {cropped_height_in_meters:.2f} m")
            rospy.loginfo(f"Pixel dimensions in meters: {resolution:.2f} m x {resolution:.2f} m")

            return cropped_width_in_meters, cropped_height_in_meters

        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call /dynamic_map service: {e}")
            return None, None
        
        
    def publish_robot_position(self):
        try:
            listener = tf.TransformListener()
            listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            robot_position = Float32MultiArray()
            robot_position.data = list(trans)  # Ajouter les coordonnées x, y, z
            self.pos_robot.publish(robot_position)
            rospy.loginfo(f"Published robot position: {trans}")

            # Publier la position du robot sur /clicked_point
            self.publish_clicked_point(trans[0], trans[1], trans[2])

        except tf.Exception as e:
            rospy.logwarn(f"Could not fetch robot position: {e}")

    def publish_clicked_point(self, x, y, z=0.0):
        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = "map"  # Adapter si nécessaire
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = z

        rospy.loginfo(f"Publishing clicked point: ({x}, {y}, {z})")
        self.clicked_point_pub.publish(point_msg)
    
    def publish_path(self):
        """
        Publie une trajectoire sous forme de Path dans RViz.
        """
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        for point in self.trajectory:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # Orientation neutre
            path.poses.append(pose)

        self.path_publisher.publish(path)
        rospy.loginfo(f"Published path with {len(self.trajectory)} points.")

    def draw_trajectory(self, image, trajectory):
        """
        Trace une trajectoire en reliant les points de la liste donnée.

        Args:
            image (numpy.ndarray): Image sur laquelle tracer la trajectoire.
            trajectory (list of tuples): Liste des points (x, y) à relier.

        Returns:
            numpy.ndarray: Image avec la trajectoire tracée.
        """
        # Vérifiez que l'image est valide
        if image is None or image.size == 0:
            rospy.logerr("L'image fournie à draw_trajectory est invalide.")
            return None

        # Vérifiez qu'il y a suffisamment de points pour tracer une trajectoire
        if len(trajectory) < 2:
            rospy.logwarn("Pas assez de points pour tracer une trajectoire.")
            return image

        # Validez les dimensions de l'image et les coordonnées des points
        height, width = image.shape[:2]
        valid_trajectory = []
        for point in trajectory:
            if 0 <= point[0] < width and 0 <= point[1] < height:
                valid_trajectory.append(point)
            else:
                rospy.logwarn(f"Point hors limites ignoré : {point}")

        # Vérifiez si la trajectoire validée contient au moins deux points
        if len(valid_trajectory) < 2:
            rospy.logerr("Aucun point valide pour tracer une trajectoire.")
            return image

        # Tracez les lignes entre les points valides
        for i in range(len(valid_trajectory) - 1):
            pt1 = (int(valid_trajectory[i][0]), int(valid_trajectory[i][1]))
            pt2 = (int(valid_trajectory[i + 1][0]), int(valid_trajectory[i + 1][1]))
            cv2.line(image, pt1, pt2, (255, 0, 0), thickness=2)  # Bleu pendant l'interaction

        rospy.loginfo(f"Trajectoire tracée avec {len(valid_trajectory)} points.")
        return image

    def get_map(self):
        try:
            rospy.loginfo("Requesting map from /dynamic_map service...")
            rospy.wait_for_service('/dynamic_map')
            map_service = rospy.ServiceProxy('/dynamic_map', GetMap)
            response = map_service()

            # Convertir la carte en format OpenCV
            map_data = np.array(response.map.data, dtype=np.int8)
            width = response.map.info.width
            height = response.map.info.height
            map_image = map_data.reshape((height, width))
            resolution = response.map.info.resolution
            origin = response.map.info.origin.position


            # Convertir les valeurs de carte
            map_image_cv = np.zeros_like(map_image, dtype=np.uint8)
            map_image_cv[map_image == 0] = 255  # Espaces libres -> blanc
            map_image_cv[map_image == 100] = 0  # Obstacles -> noir
            map_image_cv[map_image == -1] = 128  # Inconnu -> gris
            
            # Sauvegarder la carte en tant que fichier .pgm
            map_file_path = os.path.join(self.output_dir, 'map.pgm')
            cv2.imwrite(map_file_path, map_image_cv)
            rospy.loginfo(f"Map saved to {map_file_path}")

            return map_image_cv

        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to get map: {e}")
            return None

    def run(self):
        rospy.loginfo("Press 's' to fetch and process the map, or 'q' to quit.")
        while not rospy.is_shutdown():
            key = self.get_key()
            # if key == 's':
            #     rospy.loginfo("Fetching and processing map...")
            #     map_image = self.get_map()
            #     if map_image is not None:
            #         self.process_map(map_image)

            rate = rospy.Rate(1)  # Vérifier la valeur du paramètre 1 fois par seconde

            map_publi = rospy.get_param('/map_publi', False)  # Lire le paramètre avec une valeur par défaut False
            
            if map_publi:
                rospy.loginfo("map_publi is True, fetching and processing map...")
                map_image = self.get_map()
                if map_image is not None:
                    self.process_map(map_image)

                    # Une fois la carte traitée, remettre `map_publi` à False
                    rospy.set_param('/map_publi', False)
                    rospy.loginfo("Map published, resetting '/map_publi' to False.")
                break  # Sortir de la boucle après avoir traité la carte

            rate.sleep()  # Attendre avant de vérifier à nouveau

            retour_base = rospy.get_param('/retour_base', False)  # Lire le paramètre avec une valeur par défaut False
            
            elif retour_base:
                rospy.loginfo("/retour_base is True, fetching and processing map...")
                rospy.loginfo("Recording robot trajectory...")
                self.publish_robot_position()

                    # Une fois la carte traitée, remettre `map_publi` à False
                    rospy.set_param('/retour_base', False)
                    rospy.loginfo("Robot position published, resetting '/retour_base' to False.")
                break  # Sortir de la boucle après avoir traité la carte

            rate.sleep()  # Attendre avant de vérifier à nouveau
            
            # elif key == 't':
            #     rospy.loginfo("Recording robot trajectory...")
            #     self.publish_robot_position()
                    
            elif key == 'q':
                rospy.loginfo("Exiting...")
                break

    def extract_map_data(self, no_grid_image):
        """
        Extrait les données nécessaires à partir de l'image traitée.

        Args:
            no_grid_image (numpy.ndarray): Image traitée sans grille.

        Returns:
            tuple: (binary_map, resolution, origin)
        """
        # Convertir l'image en carte binaire
        grid_image_gray = cv2.cvtColor(no_grid_image, cv2.COLOR_BGR2GRAY)
        binary_map = np.where(grid_image_gray == 255, 1, 0)

        # Définir la résolution et l'origine (à adapter selon vos données)
        resolution = 0.05  # Exemple de résolution (5 cm par pixel)
        origin = (0, 0)    # Exemple d'origine (peut être ajusté selon votre carte)

        return binary_map, resolution, origin



    def trace_and_save_path(self, no_grid_image):
        """
        Permet de tracer un chemin interactif sur une image et d'enregistrer le résultat.

        Args:
            no_grid_image (numpy.ndarray): Image sans grille sur laquelle tracer le chemin.
        """
        # Vérifiez que no_grid_image est valide
        if no_grid_image is None or no_grid_image.size == 0:
            rospy.logerr("L'image fournie (no_grid_image) est vide ou invalide. Abandon.")
            return

        # Initialiser une copie pour le traçage et la trajectoire
        map_with_trajectory = no_grid_image.copy()
        trajectory = []

        def on_click(event, x, y, flags, param):
            """
            Callback pour gérer les clics sur l'image et ajouter des points à la trajectoire.
            """
            if event == cv2.EVENT_LBUTTONDOWN:
                rospy.loginfo(f"Point ajouté : ({x}, {y})")
                trajectory.append((x, y))
                rospy.loginfo(f"Trajectoire actuelle : {trajectory}")

                # Mettre à jour l'image avec le chemin tracé (temporairement en bleu)
                updated_image = self.draw_trajectory(map_with_trajectory.copy(), trajectory)

                # Vérifiez que l'image mise à jour est valide avant affichage
                if updated_image is None or updated_image.size == 0:
                    rospy.logerr("L'image mise à jour est invalide après traçage.")
                    return

                # Affichez l'image mise à jour
                rospy.loginfo("Affichage de l'image mise à jour.")
                cv2.imshow("Interactive Map", updated_image)

        # Configurer la fenêtre et les interactions
        cv2.namedWindow("Interactive Map")
        cv2.setMouseCallback("Interactive Map", on_click)

        while True:
            # Affichez l'image initiale
            cv2.imshow("Interactive Map", map_with_trajectory)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):  # Quitter sans sauvegarder
                rospy.loginfo("Fermeture sans sauvegarde.")
                break
            elif key == ord('f'):  # Enregistrer la carte avec le chemin tracé
                if len(trajectory) > 0:
                    # Dessinez le chemin final en rouge sur l'image
                    rospy.loginfo("Traçage du chemin final en rouge.")
                    final_image = self.draw_trajectory(map_with_trajectory.copy(), trajectory)
                    for i in range(len(trajectory) - 1):
                        pt1 = (trajectory[i][0], trajectory[i][1])
                        pt2 = (trajectory[i + 1][0], trajectory[i + 1][1])
                        cv2.line(final_image, pt1, pt2, (0, 0, 255), thickness=2)

                    # Enregistrez l'image finale
                    output_path = os.path.join(self.output_dir, 'map_with_path.png')
                    cv2.imwrite(output_path, final_image)
                    rospy.loginfo(f"Carte avec chemin enregistrée sous : {output_path}")
                else:
                    rospy.logwarn("Aucun chemin n'a été tracé. Rien à sauvegarder.")
                break

        cv2.destroyAllWindows()


    def process_map(self, map_image):
        try:
            rospy.loginfo(f"Map image loaded: {map_image.shape}")

            # Identifier et rogner la zone avec le maximum de pixels noirs
            # cropped_image = self.crop_to_black_area(map_image)
            cropped_image, x_min, y_min = self.crop_to_black_area(map_image)

            rospy.loginfo("Requesting map from /dynamic_map service...")
            rospy.wait_for_service('/dynamic_map')
            map_service = rospy.ServiceProxy('/dynamic_map', GetMap)
            response = map_service()

            # Convertir la carte en format OpenCV
            map_data = np.array(response.map.data, dtype=np.int8)
            width = response.map.info.width
            height = response.map.info.height
            map_image = map_data.reshape((height, width))
            resolution = response.map.info.resolution
            # Extraire les coordonnées de l'origine
            origin = (response.map.info.origin.position.x, response.map.info.origin.position.y)

            # Extraire les coordonnées de l'origine de la carte entière
            origin_x = response.map.info.origin.position.x
            origin_y = response.map.info.origin.position.y

            rospy.loginfo(f"Original origin: ({origin_x}, {origin_y})")
            rospy.loginfo(f"x_min, y_min in pixels: ({x_min}, {y_min})")
            rospy.loginfo(f"Resolution: {resolution}")

            # Calculer la nouvelle origine après rognage
            offset_x = x_min * resolution
            offset_y = y_min * resolution

            new_origin_x = origin_x + offset_x
            new_origin_y = origin_y + offset_y

            rospy.loginfo(f"New origin (before scaling): ({new_origin_x}, {new_origin_y})")


            # Grossir l'image
            scaled_image = cv2.resize(
                cropped_image,
                None,
                fx=self.scale_factor,
                fy=self.scale_factor,
                interpolation=cv2.INTER_NEAREST
            )

            height, width = cropped_image.shape
            rospy.loginfo(f"height and width: ({height}, {width})")

            # Calculer les coordonnées des coins en mètres
            # top_left = (new_origin_x, new_origin_y)
            # top_right = (new_origin_x + width * resolution, new_origin_y)
            # bottom_left = (new_origin_x, new_origin_y + height * resolution)

            top_left = (new_origin_x, new_origin_y + height * resolution)
            top_right = (new_origin_x + width * resolution, new_origin_y + height*resolution)
            bottom_left = (new_origin_x, new_origin_y)

            # Préparer le message
            corners_msg = Float64MultiArray()
            corners_msg.data = [
                top_left[0], top_left[1],
                top_right[0], top_right[1],
                bottom_left[0], bottom_left[1]
            ]

            # Publier le message
            self.corners_publisher.publish(corners_msg)
            rospy.loginfo(f"Published corners: Top Left {top_left}, Top Right {top_right}, Bottom Left {bottom_left}")

            
            #rospy.loginfo(f"New origin (after scaling): ({new_origin_x}, {new_origin_y})")

            height, width = scaled_image.shape
            rospy.loginfo(f"Scaled image dimensions: {width} x {height}")

            # Publier les coordonnées des coins de la carte rognée
            # self.publish_corners(scaled_image, resolution, origin)
            #self.publish_corners(scaled_image, resolution * self.scale_factor, (new_origin_x, new_origin_y))

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
            grid_image_gray = cv2.cvtColor(no_grid_image, cv2.COLOR_BGR2GRAY)
            binary_map = np.where(grid_image_gray == 255, 1, 0)
            rows, cols = binary_map.shape
            print(f"Number of rows: {rows}, Number of columns: {cols}")

            # Publier le tableau binaire
            self.publish_binary_map(binary_map)

            # Sauvegarder le tableau binaire dans un fichier texte
            binary_output_file = os.path.join(self.output_dir, 'obstacle_map.txt')
            np.savetxt(binary_output_file, binary_map, fmt='%d', delimiter='')
            rospy.loginfo(f"Obstacle map saved to {binary_output_file}")

            # **Tracer un chemin interactif sur l'image rognée sans grille**
            # rospy.loginfo("Launching interactive path tracing...")
            # self.trace_and_save_path(no_grid_image)

        except Exception as e:
            rospy.logerr(f"Failed to process map: {e}")


    def crop_to_black_area(self, image):
        """
        Identifie et rogne la zone contenant le maximum de pixels noirs dans l'image.
        Retourne aussi les coordonnées x_min et y_min du coin supérieur gauche.
        """
        black_pixels = np.where(image == 0)

        if black_pixels[0].size == 0 or black_pixels[1].size == 0:
            rospy.logwarn("No black pixels detected in the image.")
            return image, 0, 0  # Retourne l'image complète avec (x_min=0, y_min=0) si aucun noir n'est détecté

        # Trouver les limites de la zone noire
        min_row, max_row = np.min(black_pixels[0]), np.max(black_pixels[0])
        min_col, max_col = np.min(black_pixels[1]), np.max(black_pixels[1])

        rospy.loginfo(f"Cropping to rectangle: ({min_row}, {min_col}) - ({max_row}, {max_col})")

        # Rogner l’image
        cropped_image = image[min_row:max_row + 1, min_col:max_col + 1]

        # min_col correspond à x_min (abscisse) et min_row correspond à y_min (ordonnée)
        x_min, y_min = min_col, min_row
        print(f"X_min: {x_min}, Y_min: {y_min}")

        return cropped_image, x_min, y_min


    def expand_black_pixels(self, image):
        """
        Étend les pixels noirs en fonction de leurs voisins pour combler les espaces proches.
        """
        denoised_image = cv2.medianBlur(image, 3)
        _, binary_image = cv2.threshold(denoised_image, 50, 255, cv2.THRESH_BINARY_INV)

        dilation_size = 15
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
        grid_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        no_grid_image = grid_image.copy()
        step = self.grid_step

        obstacle_threshold = 0.1

        for y in range(0, image.shape[0], step):
            for x in range(0, image.shape[1], step):
                roi = image[y:y + step, x:x + step]

                if np.sum(roi == 0) / roi.size > obstacle_threshold:
                    cv2.rectangle(
                        grid_image,
                        (x, y),
                        (x + step, y + step),
                        (0, 0, 255),
                        -1
                    )

                cv2.rectangle(
                    grid_image,
                    (x, y),
                    (x + step, y + step),
                    (255, 255, 255),
                    1
                )

        return no_grid_image, grid_image

    def publish_binary_map(self, binary_map):
        """
        Publie le tableau binaire (converti en tableau Python) sur un topic ROS.
        """
        msg = Int32MultiArray()

        rows, cols = binary_map.shape
        msg.layout.dim.append(MultiArrayDimension(label="rows", size=rows, stride=rows * cols))
        msg.layout.dim.append(MultiArrayDimension(label="cols", size=cols, stride=cols))
        msg.data = binary_map.flatten().tolist()

        self.map_publisher.publish(msg)
        rospy.loginfo(f"Published binary map of size {rows}x{cols} on 'binary_map_topic'")


if __name__ == '__main__':
    try:
        ImageConverter()
    except rospy.ROSInterruptException:
        rospy.loginfo("Image converter node terminated.")

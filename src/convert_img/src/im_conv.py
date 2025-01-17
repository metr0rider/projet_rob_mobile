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
from std_msgs.msg import Int32MultiArray, MultiArrayDimension, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


class ImageConverter:
    def __init__(self):
        # Initialisation de ROS
        rospy.init_node('im_conv', anonymous=True)

        # Publisher pour le tableau binaire
        self.map_publisher = rospy.Publisher('binary_map_topic', Int32MultiArray, queue_size=10)
        
        # Publisher pour la position du robot
        self.pos_robot = rospy.Publisher('pos_robot', Float32MultiArray, queue_size=10)

        # Publisher pour le chemin
        self.path_publisher = rospy.Publisher('path', Path, queue_size=10)

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


    # def get_map_dimensions(self):
    #     """
    #     Récupère les dimensions de la carte en pixels et en mètres.
    #     """
    #     rospy.wait_for_service('/dynamic_map')
    #     try:
    #         # Appeler le service
    #         map_service = rospy.ServiceProxy('/dynamic_map', GetMap)
    #         response = map_service()

    #         # Extraire les informations de la carte
    #         resolution = response.map.info.resolution
    #         width = response.map.info.width
    #         height = response.map.info.height

    #         # Calculer les dimensions en mètres
    #         width_in_meters = width * resolution
    #         height_in_meters = height * resolution

    #         dim_meters_1pix = width_in_meters/width

    #         rospy.loginfo(f"Map dimensions in pixels: {width} x {height}")
    #         rospy.loginfo(f"Map dimensions in meters: {width_in_meters:.2f} m x {height_in_meters:.2f} m")
    #         rospy.loginfo(f"Pixel dimensions in meters: {dim_meters_1pix:.2f} m x {dim_meters_1pix:.2f} m")

    #         return width_in_meters, height_in_meters

    #     except rospy.ServiceException as e:
    #         rospy.logerr(f"Failed to call /dynamic_map service: {e}")
    #         return None, None
    
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
        except tf.Exception as e:
            rospy.logwarn(f"Could not fetch robot position: {e}")

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
            color (tuple): Couleur de la trajectoire en BGR (par défaut : rouge).
            thickness (int): Épaisseur de la ligne (par défaut : 2).

        Returns:
            numpy.ndarray: Image avec la trajectoire tracée.
        """
        color = (0, 0, 255)
        thickness = 2
        if len(trajectory) < 2:
            return image  # Pas assez de points pour tracer une trajectoire.

        # for i in range(len(trajectory) - 1):
        #     pt1 = trajectory[i]
        #     pt2 = trajectory[i + 1]
        #     cv2.line(image, pt1, pt2, color, thickness)
        #     print('x')
        for i in range(len(trajectory) - 1):
            pt1 = (int(trajectory[i][0]), int(trajectory[i][1]))
            pt2 = (int(trajectory[i + 1][0]), int(trajectory[i + 1][1]))
            print(f"Drawing line from {pt1} to {pt2}")  # Débogage
            cv2.line(image, pt1, pt2, color, thickness)

        return image
    
    def on_click(self, event, x, y, flags, param):
        """
        Callback pour gérer les clics sur l'image et ajouter des points à la trajectoire.
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            rospy.loginfo(f"Point clicked: ({x}, {y})")
            self.trajectory.append((x, y))
            rospy.loginfo(f"Current trajectory: {self.trajectory}")
            
            # Convertir en CV_8U si nécessaire
            if self.map_image.dtype == np.int8:  # Vérifie si l'image est en CV_8S
                self.map_image = cv2.convertScaleAbs(self.map_image)

            # Vérifiez et préparez l'image pour dessiner
            if len(self.map_image.shape) == 2:  # Image en niveaux de gris
                self.map_image_with_trajectory = cv2.cvtColor(self.map_image, cv2.COLOR_GRAY2BGR)
            else:  # Image déjà en couleur
                self.map_image_with_trajectory = self.map_image.copy()
            
            # Tracez la trajectoire
            self.map_image_with_trajectory = self.draw_trajectory(self.map_image_with_trajectory, self.trajectory)

            # Affichez l'image avec la trajectoire
            cv2.imshow("Cropped Map", self.map_image_with_trajectory)
            cv2.waitKey(1)  # Nécessaire pour actualiser l'affichage

            # Publiez la trajectoire
            self.publish_path()


    

    # def get_map(self):
    #     try:
    #         rospy.loginfo("Requesting map from /dynamic_map service...")
    #         rospy.wait_for_service('/dynamic_map')
    #         map_service = rospy.ServiceProxy('/dynamic_map', GetMap)
    #         response = map_service()

    #         # Convertir la carte en format OpenCV
    #         map_data = np.array(response.map.data, dtype=np.int8)
    #         width = response.map.info.width
    #         height = response.map.info.height
    #         map_image = map_data.reshape((height, width))
    #         resolution = response.map.info.resolution
    #         origin = response.map.info.origin.position


    #         # Convertir les valeurs de carte
    #         map_image_cv = np.zeros_like(map_image, dtype=np.uint8)
    #         map_image_cv[map_image == 0] = 255  # Espaces libres -> blanc
    #         map_image_cv[map_image == 100] = 0  # Obstacles -> noir
    #         map_image_cv[map_image == -1] = 128  # Inconnu -> gris
            
    #         # Ajouter les trajectoires, le point de départ et les points d'arrêt
    #         # self.overlay_trajectories(map_image_cv, resolution, origin)
            
    #         # Sauvegarder la carte en tant que fichier .pgm
    #         map_file_path = os.path.join(self.output_dir, 'map.pgm')
    #         cv2.imwrite(map_file_path, map_image_cv)
    #         rospy.loginfo(f"Map saved to {map_file_path}")

    #         return map_image_cv

    #     except rospy.ServiceException as e:
    #         rospy.logerr(f"Failed to get map: {e}")
    #         return None

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

            # Afficher l'image et enregistrer les clics
            cv2.namedWindow("Map")
            cv2.setMouseCallback("Map", self.on_click)

            while True:
                cv2.imshow("Map", map_image_cv)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break

            cv2.destroyAllWindows()

            return map_image_cv

        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to get map: {e}")
            return None

    def get_cropped_map(self):
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

            # Rogner la carte sur les zones contenant des pixels noirs
            cropped_image = self.crop_to_black_area(map_image)

            # Sauvegarder l'image rognée originale et une copie pour tracer la trajectoire
            self.map_image = cropped_image  # Sauvegarder l'image rognée originale
            self.map_image_with_trajectory = self.map_image.copy()

            # Sauvegarder la carte rognée
            map_file_path = os.path.join(self.output_dir, 'cropped_map.pgm')
            cv2.imwrite(map_file_path, cropped_image)
            rospy.loginfo(f"Cropped map saved to {map_file_path}")

            # Afficher l'image rognée et enregistrer les clics
            cv2.namedWindow("Cropped Map")
            cv2.setMouseCallback("Cropped Map", self.on_click)

            while True:
                cv2.imshow("Cropped Map", cropped_image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break

            cv2.destroyAllWindows()

            return cropped_image

        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to get map: {e}")
            return None
            

    def run(self):
        rospy.loginfo("Press 's' to fetch and process the map, or 'q' to quit.")
        while not rospy.is_shutdown():
            key = self.get_key()
            if key == 's':
                rospy.loginfo("Fetching and processing map...")
                map_image = self.get_cropped_map()
                if map_image is not None:
                    self.process_map(map_image)
            
            elif key == 't':
                rospy.loginfo("Recording robot trajectory...")
                # self.publish_robot_position()

            elif key == 'd':
                rospy.loginfo("Displaying map dimensions...")
                width_m, height_m = self.get_map_dimensions()
                if width_m and height_m:
                    rospy.loginfo(f"Map dimensions: {width_m:.2f} m x {height_m:.2f} m")
                    
            elif key == 'q':
                rospy.loginfo("Exiting...")
                break


    def process_map(self, map_image):
        try:
            rospy.loginfo(f"Map image loaded: {map_image.shape}")

            # Récupérer la taille d'un pixel en mètre depuis le topic dynamic_map
            resolution = rospy.get_param('/dynamic_map/info/resolution', None)
            if resolution is None:
                rospy.logwarn("Unable to retrieve resolution from /dynamic_map/info/resolution. Defaulting to 1.0.")
                resolution = 1.0
            rospy.loginfo(f"Pixel size in meters: {resolution}")

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

        except Exception as e:
            rospy.logerr(f"Failed to process map: {e}")

    def crop_to_black_area(self, image):
        """
        Identifie et rogner la zone contenant le maximum de pixels noirs dans l'image.
        """
        black_pixels = np.where(image == 0)

        if black_pixels[0].size == 0 or black_pixels[1].size == 0:
            rospy.logwarn("No black pixels detected in the image.")
            return image

        min_row, max_row = np.min(black_pixels[0]), np.max(black_pixels[0])
        min_col, max_col = np.min(black_pixels[1]), np.max(black_pixels[1])

        rospy.loginfo(f"Cropping to rectangle: ({min_row}, {min_col}) - ({max_row}, {max_col})")
        cropped_image = image[min_row:max_row + 1, min_col:max_col + 1]
        return cropped_image

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

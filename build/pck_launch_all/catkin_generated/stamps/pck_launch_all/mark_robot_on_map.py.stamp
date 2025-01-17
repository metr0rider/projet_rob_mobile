#!/usr/bin/env python3

import rospy
<<<<<<< HEAD
import tf2_ros
=======
import tf
>>>>>>> b157bc9fbefe0d5e0a2bb8ea58a4cca1ac17fbfd
import cv2
import yaml
import numpy as np

def draw_robot_on_map(map_image_path, map_yaml_path, robot_position, output_image_path):
    # Charger l'image de la carte
    map_image = cv2.imread(map_image_path, cv2.IMREAD_UNCHANGED)
    if map_image is None:
        rospy.logerr("Impossible de charger l'image de la carte.")
        return

    # Charger les paramètres de la carte
    with open(map_yaml_path, 'r') as yaml_file:
        map_data = yaml.safe_load(yaml_file)

    resolution = map_data['resolution']
    origin = map_data['origin']

    # Convertir la position du robot (en coordonnées réelles) en coordonnées pixel
    x_pixel = int((robot_position[0] - origin[0]) / resolution)
    y_pixel = int((robot_position[1] - origin[1]) / resolution)

    # Inverser l'axe Y pour correspondre à l'image
    y_pixel = map_image.shape[0] - y_pixel

    # Dessiner la position du robot sur la carte
    cv2.circle(map_image, (x_pixel, y_pixel), radius=5, color=(0, 0, 255), thickness=-1)

    # Sauvegarder l'image résultante
    cv2.imwrite(output_image_path, map_image)
    rospy.loginfo(f"Carte avec la position du robot sauvegardée dans {output_image_path}")

def get_robot_position():
    # Initialisation de ROS
    rospy.init_node('robot_position_marker', anonymous=True)

<<<<<<< HEAD
    # Création du buffer et du listener TF2
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    try:
        # Attendre une transformation disponible
        rospy.loginfo("Attente de la transformation entre '/map' et '/base_link'...")
        trans = tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(10.0))

        # Récupérer les coordonnées x, y, z
        robot_position = trans.transform.translation
        return (robot_position.x, robot_position.y, robot_position.z)
    except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
=======
    # Attente des données de TF
    listener = tf.TransformListener()
    listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(10.0))

    try:
        (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        return trans  # Retourne les coordonnées x, y, z
    except tf.Exception as e:
>>>>>>> b157bc9fbefe0d5e0a2bb8ea58a4cca1ac17fbfd
        rospy.logerr(f"Impossible de récupérer la position du robot : {e}")
        return None

if __name__ == '__main__':
    map_image_path = "/home/projet_rob_mobile/map.pgm"
    map_yaml_path = "/home/projet_rob_mobile/map.yaml"
    output_image_path = "/home/projet_rob_mobile/map_with_robot.pgm"

<<<<<<< HEAD
    # Obtenir la position du robot
    robot_position = get_robot_position()
    if robot_position:
        # Dessiner la position du robot sur la carte
=======
    robot_position = get_robot_position()
    if robot_position:
>>>>>>> b157bc9fbefe0d5e0a2bb8ea58a4cca1ac17fbfd
        draw_robot_on_map(map_image_path, map_yaml_path, robot_position, output_image_path)


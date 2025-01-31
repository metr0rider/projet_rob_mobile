#!/usr/bin/env python3
import rospy
import numpy as np
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import random

class RRTExploration:
    def __init__(self):
        rospy.init_node('rrt_explorer', anonymous=True)

        # Paramètres de RRT
        self.step_size = 0.5  # Distance entre les nœuds
        self.unknown_threshold = -1  # Valeur des zones inconnues dans la carte
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.exploring = True

        # Abonnements ROS
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Publisher pour le contrôle du robot
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.loginfo("Exploration autonome avec RRT (via /cmd_vel) démarrée !")

    def map_callback(self, map_msg):
        """ Récupère la carte et prépare les données pour RRT. """
        rospy.loginfo("Carte reçue, mise à jour des données.")
        self.map_data = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        self.map_resolution = map_msg.info.resolution
        self.map_origin = map_msg.info.origin

    def scan_callback(self, scan_msg):
        """ Vérifie la présence d'obstacles proches et ajuste la trajectoire si nécessaire. """
        ranges = np.array(scan_msg.ranges)
        ranges = np.where(np.isnan(ranges), scan_msg.range_max, ranges)

        min_distance = np.min(ranges)
        if min_distance < 0.5:  # Si un obstacle est trop proche, annule le mouvement
            rospy.logwarn("Obstacle détecté, recalcul de la trajectoire...")
            self.avoid_obstacle()

    def avoid_obstacle(self):
        """ Manœuvre d'évitement d'obstacles. """
        twist = Twist()
        twist.linear.x = -0.2  # Reculer légèrement
        twist.angular.z = random.choice([-1.0, 1.0])  # Tourner aléatoirement
        self.cmd_pub.publish(twist)
        rospy.sleep(1)

        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_pub.publish(twist)
        rospy.sleep(1)

    def sample_free_space(self):
        """ Génère un point aléatoire dans une zone inexplorée. """
        height, width = self.map_data.shape
        while True:
            x_pixel = random.randint(0, width - 1)
            y_pixel = random.randint(0, height - 1)

            if self.map_data[y_pixel, x_pixel] == self.unknown_threshold:
                x_real = self.map_origin.position.x + x_pixel * self.map_resolution
                y_real = self.map_origin.position.y + y_pixel * self.map_resolution
                return x_real, y_real

    def generate_rrt_path(self, start_x, start_y, goal_x, goal_y):
        """ Génère un chemin avec RRT entre un point de départ et un objectif. """
        tree = [(start_x, start_y)]
        while rospy.get_param('/explo_auto', False):
            rospy.loginfo("Génération du chemin RRT...")
            x_rand, y_rand = self.sample_free_space()
            nearest_node = min(tree, key=lambda node: np.linalg.norm(np.array(node) - np.array([x_rand, y_rand])))

            direction = np.array([x_rand, y_rand]) - np.array(nearest_node)
            direction = direction / np.linalg.norm(direction) * self.step_size
            new_node = (nearest_node[0] + direction[0], nearest_node[1] + direction[1])

            if self.map_data[int((new_node[1] - self.map_origin.position.y) / self.map_resolution), 
                             int((new_node[0] - self.map_origin.position.x) / self.map_resolution)] != 100:
                tree.append(new_node)

            if np.linalg.norm(np.array(new_node) - np.array([goal_x, goal_y])) < self.step_size:
                return tree
        return []

    def move_along_path(self, path):
        """ Fait suivre au robot le chemin généré par RRT en publiant sur /cmd_vel. """
        for i in range(len(path) - 1):
            current = np.array(path[i])
            next_point = np.array(path[i + 1])

            direction = next_point - current
            angle = np.arctan2(direction[1], direction[0])

            twist = Twist()
            twist.linear.x = 0.3  # Vitesse en ligne droite
            twist.angular.z = angle * 0.5  # Correction d'orientation
            self.cmd_pub.publish(twist)

            rospy.sleep(1)

        # Arrêt du robot
        self.cmd_pub.publish(Twist())

    def run(self):
        """ Boucle principale d'exploration. """
        rate = rospy.Rate(0.1)  # Mise à jour toutes les 10 secondes
        while not rospy.is_shutdown() and self.exploring:
            if self.map_data is None:
                rospy.loginfo("Attente de la carte...")
                rate.sleep()
                continue

            start_x, start_y = self.sample_free_space()
            goal_x, goal_y = self.sample_free_space()

            path = self.generate_rrt_path(start_x, start_y, goal_x, goal_y)

            if path:
                rospy.loginfo("Suivi du chemin généré...")
                self.move_along_path(path)

            rate.sleep()

if __name__ == "__main__":
    try:
        explorer = RRTExploration()
        explorer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exploration interrompue.")

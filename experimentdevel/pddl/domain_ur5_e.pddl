 
(define (domain ur5_e)
  
(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types waypoint robot)
  
(:predicates
    (robot_at ?v - robot ?wp - waypoint)
	(visited ?wp - waypoint)
    (is_wp_reachable ?wp - waypoint ?v - robot))
  
(:durative-action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:duration ( = ?duration 50)
	:condition (at start (robot_at ?v ?from))
	:effect (and
		(at end (visited ?to))
		(at end (robot_at ?v ?to))
		(at start (not (robot_at ?v ?from)))))
)
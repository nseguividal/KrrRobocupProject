


; (define (problem test2-obstacle)
;   (:domain task3)

;   (:objects
;     kitchen bedroom - room
;     scan_kitchen scan_bedroom loc_box - location
;     box - item
;   )

;   (:init
;     ;; Estado del robot
;     (free_gripper)
;     (agent_at scan_kitchen)
    
;     ;; Estado de las habitaciones (debe terminar la cocina para ir al dormitorio)
;     (tidying kitchen)
;     (untidy bedroom)
    
;     (scan_loc scan_kitchen)
;     (scan_loc scan_bedroom)
;     (location_in_room scan_kitchen kitchen)
;     (location_in_room scan_bedroom bedroom)
;     (location_in_room loc_box kitchen)

;     ;; ¡EL OBSTÁCULO! 
;     ;; Fíjate que NO ponemos (door_clear kitchen bedroom)
;     (item_at box loc_box)
;     (door_blocked_by box kitchen bedroom)
;   )

;   (:goal (and
;     ;; Nuestro único objetivo es llegar al dormitorio (Exploración)
;     (agent_at scan_bedroom)
;   ))
; )





; (define (problem test1-inspect)
;   (:domain task3)

;   (:objects
;     kitchen - room
;     scan_kitchen loc_obj_1 loc_obj_2 - location
;     obj_unknown obj_unknown2 - item
;   )

;   (:init
;     ;; Estado del robot
;     (free_gripper)
;     (agent_at scan_kitchen)
    
;     ;; Estado de la habitación
;     (tidying kitchen)
;     (scan_loc scan_kitchen)
;     (location_in_room scan_kitchen kitchen)
;     (location_in_room loc_obj_1 kitchen)
; 	  (location_in_room loc_obj_2 kitchen)

;     ;; El objeto misterioso en el suelo
;     (item_at obj_unknown loc_obj_1)
; 	  (item_at obj_unknown2 loc_obj_2)

;     (standard_item obj_unknown)
;     (standard_item obj_unknown2)
;   )

;   (:goal (and
;     ;; Queremos saber qué es el objeto (Fase 1 de tu script Python)
;     (inspected obj_unknown)
; 	  (inspected obj_unknown2)
;     (free_gripper)
;   ))
; )

(define (problem tidy-task2)
  (:domain task3)

  (:objects

    ;; Items detected in the house
    obj_0_kitchen - item
    obj_1_kitchen - item
    obj_0_office - item

    ;; Scan / waypoint locations for each room
    scan_kitchen      - location
    scan_bedroom      - location
    scan_office       - location
    scan_living_room  - location

    ;; Initial object locations
    loc_obj_0_kitchen - location
    loc_obj_1_kitchen - location
    loc_obj_0_office - location

    loc_drop_dummy - location

    ;; Rooms
    kitchen     - room
    bedroom     - room
    office      - room
    living_room - room
  )

  (:init
    ;; Robot starts empty-handed at the kitchen scan location
    (free_gripper)
    (agent_at scan_kitchen)

    ;; Room traversal state (identical to Task 1)
    (tidy bedroom)
    (untidy kitchen)
    (tidy office)
    (untidy living_room)
    (tidying kitchen)

    ;; Scan waypoints
    (scan_loc scan_kitchen)
    (scan_loc scan_bedroom)
    (scan_loc scan_office)
    (scan_loc scan_living_room)

    ;; Location → room mapping
    (location_in_room scan_kitchen      kitchen)
    (location_in_room scan_bedroom      bedroom)
    (location_in_room scan_office       office)
    (location_in_room scan_living_room  living_room)

    ;; Object locations belong to their source rooms
    (location_in_room loc_obj_0_kitchen kitchen)
    (location_in_room loc_obj_1_kitchen kitchen)
    (location_in_room loc_obj_0_office office)


    ;; Initial item positions (discovered during pre-scan)
    (item_at obj_0_kitchen loc_obj_0_kitchen)
    (item_at obj_1_kitchen loc_obj_1_kitchen)
    (item_at obj_0_office loc_obj_0_office)

    ;; TASK 3 REQUIREMENT: Doors start clear
    (door_clear kitchen bedroom)
    (door_clear kitchen living_room)

    (door_clear bedroom office)
    (door_clear bedroom kitchen)

    (door_clear office living_room)
    (door_clear office bedroom)

    (door_clear living_room office)
    (door_clear living_room kitchen)


    (standard_item obj_0_kitchen)
    (standard_item obj_1_kitchen)
    (standard_item obj_0_office)
    
  )

  (:goal (and
    ;; All rooms visited and tidied

    ;; Semantic placement goals (resolved from TypeDB inference rule)
    (on_drop_loc obj_0_kitchen)
    (on_drop_loc obj_1_kitchen)
    (on_drop_loc obj_0_office)
    
  ))

)

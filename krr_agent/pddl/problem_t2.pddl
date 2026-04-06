(define (problem tidy-task2)
  (:domain task2)

  (:objects

    ;; Items detected in the house
    obj_0_kitchen - item
    obj_1_kitchen - item
    obj_0_bedroom - item
    obj_1_bedroom - item
    obj_0_office - item
    obj_1_living_room - item

    ;; Scan / waypoint locations for each room
    scan_kitchen      - location
    scan_bedroom      - location
    scan_office       - location
    scan_living_room  - location

    ;; Initial object locations
    loc_obj_0_kitchen      - location
    loc_obj_1_kitchen      - location
    loc_obj_0_bedroom      - location
    loc_obj_1_bedroom      - location
    loc_obj_0_office       - location
    loc_obj_1_living_room  - location

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
    (untidy kitchen)
    (untidy bedroom)
    (untidy office)
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
    (location_in_room loc_obj_0_bedroom bedroom)
    (location_in_room loc_obj_1_bedroom bedroom)
    (location_in_room loc_obj_0_office office)
    (location_in_room loc_obj_1_living_room living_room)


    ;; Initial item positions (discovered during pre-scan)
    (item_at obj_0_kitchen loc_obj_0_kitchen)
    (item_at obj_1_kitchen loc_obj_1_kitchen)
    (item_at obj_0_bedroom loc_obj_0_bedroom)
    (item_at obj_1_bedroom loc_obj_1_bedroom)
    (item_at obj_0_office loc_obj_0_office)
    (item_at obj_1_living_room loc_obj_1_living_room)


    ;; add these only if you restore adjacency in domain
    (adjacent kitchen bedroom)
    (adjacent bedroom kitchen)
    (adjacent kitchen living_room)
    (adjacent living_room kitchen)
    (adjacent bedroom office)
    (adjacent office bedroom)
    (adjacent living_room office)
    (adjacent office living_room)

  )

  (:goal (and
    ;; All rooms visited and tidied
    (tidy kitchen)
    (tidy bedroom)
    (tidy office)
    (tidy living_room)

    (on_drop_loc obj_0_kitchen)
    (on_drop_loc obj_1_kitchen)
    (on_drop_loc obj_0_bedroom)
    (on_drop_loc obj_1_bedroom)
    (on_drop_loc obj_0_office)
    (on_drop_loc obj_1_living_room)
    
  ))

)

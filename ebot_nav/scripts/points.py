
way_points_in = [
            #Enter Start Zone
            [
            ],

            #Meeting Room In to Dropbox to Pickup Point
            [
            ( 6.9, 2.6, 0.0, 0.010), #12   Meeting DropBox
            ( 7.6, 2.4, 0.0, 0.007), #13   Meeting Pickup
            ],

            #Research Room Entry to Dropbox
            [
            ( 11.200, 10.010, -0.018, -1.00), #16  Reaserch DropBox along length
            ],

            #Store Room Pick up
            [
            ( 26.065, -2.714, -0.891, 0.454),  #19  Store Pickup 1
            ( 25.8179, -3.3344, -0.871, 0.490), #21   Store Pickup 2
            ],

            #Pantry In to Table 1 -> 2
            [
            ( 13.035, 0.99, -0.702, 0.711), #1  Pantry Intermediate outside door
            ( 13.035, -0.1, -0.702, 0.711), #2  Pantry Intermediate inside door
            ( 13.035, -0.1, -0.182, 0.98), #3   Pantry Intermediate inside door orientation
            ( 14.695, -0.75, -0.718, 0.695), #4  Pantry Pickup Table 1
            ( 14.615, -0.62, 1, 0.0008), #5   Pantry Pickup Table 1 - Orientation to Table2
            ( 11.15, -1.32, -0.709, -0.70), #6   Pantry Pickup Table 2 Position 1

            ( 11.15, -1.32, 0.3824, 0.92395), #7  Pantry Pick.70up Table 2 Position 1 Orientation
            ],

            #Conference Room Entry to Dropbox
            [
            ( 5.156, 0.861, -0.706, 0.7082), #24   Conference Intermediate CV
            ( 5.070, -0.7, -0.9126, 0.4087), #25  Conference DropBox CV
            ]
            ]

way_points_out = [
            [
            ],

            #Exit Meeting Room
            [
                        ],
            #Research Exit to Store room
            [
            ( 14.6, 10.1097, -0.011123, -0.9936), #17  Reaserch DropBox Intermediatie
            ( 15.0, 3.9, -0.623, 0.781) #18   Research lab to store room Intermediate
            ],

            #Store Room Exit from Table 1
            [
            ( 25.8179, -3.3344, 0.894, -0.448), #22    Store Pickup 2 Orientation to Exit
            ( 26.065, -2.714, -0.994, 0.0),  #20   Store Pickup 1 Orientation to Exit
            ( 16.7, 1.0, 1, 0.0) #23   Store Pickup 2 out
            ],

            #Store Room Exit from Table 2
            [
            ( 25.8179, -3.3344, 0.894, -0.448), #22    Store Pickup 2 Orientation to Exit
            ( 26.065, -2.714, -0.994, 0.0),  #20   Store Pickup 1 Orientation to Exit
            ( 16.7, 1.0, 1, 0.0) #23   Store Pickup 2 out
            ],
            #Exit Pantry
            [
            ( 13.2159, -0.604, -0.7577, -0.650), #8  Pantry Out OIntermediate
            ( 13.0, 0.8, -0.719, -0.694) #9  Pantry Out OIntermediate
            ],
            #Exit Conference Room
            [
            ( 5.070, -0.7, -0.7583, -0.6518), #26  Conference Intermediate out CV
            ( 5.18, 0.1, -0.7583, -0.6518), #27   Conference Intermediate out CV 1
            ( 5.25, 0.65, -0.7583, -0.6518) #27   Conference Intermediate out CV 2
            ]
            ]

Intermediate = []

DropBox = [
            #Entrance to Meeting Room Dropbox
            [
            ()
            ],
            #Entrance to Meeting Room Dropbox
            [
            ()
            ],
            #Entrance to Meeting Room Dropbox
            [
            ()
            ]
            ]


for i in range(len(way_points)):
    print(len(way_points[i]))

























way_points = [
            # (3.6, 0.85,0,1), #0  enter the hallway point

            #Object 1
            # (13.035, 0.99, -0.702, 0.711), #1  Pantry Intermediate outside door
            # (13.035, -0.1, -0.702, 0.711), #2  Pantry Intermediate inside door
            # (13.035, -0.1, -0.182, 0.98), #3   Pantry Intermediate inside door orientation
            # (14.695, -0.75, -0.718, 0.695), #4  Pantry Pickup Table 1

            (14.615, -0.62, 1, 0.0008), #5   Pantry Pickup Table 1 - Orientation to Table2
            # (11.15, -1.32, -0.709, -0.70), #6   Pantry Pickup Table 2 Position 1
            #
            # (11.15, -1.32, 0.3824, 0.92395), #7  Pantry Pick.70up Table 2 Position 1 Orientation
            # (13.2159, -0.604, -0.7577, -0.650), #8  Pantry Out OIntermediate
            # (13.0, 0.8, -0.719, -0.694), #9  Pantry Out OIntermediate

            # ( 8.64, 1.148, 0.7063, 0.7078), #10   Meeting Intermediate CV
            # ( 8.64, 2.4, 1.0, 0.0003), #11  Meeting Intermediate 2 CV
            # ( 6.9, 2.6, 0.0, 0.010), #12   Meeting DropBox
            # #Object 2
            # ( 7.6, 2.4, 0.0, 0.007), #13   Meeting Pickup
            # ( 8.5, 2.175, -0.7068, 0.7073), #14   Meeting Intermediate out CV
            # ( 8.638, 1.148, -0.7068, 0.7073), #15   Meeting Intermediate CV

            # (11.200, 10.010, -0.018, -1.00), #16  Reaserch DropBox along length
            # (14.6, 10.1097, -0.011123, -0.9936), #17  Reaserch DropBox Intermediatie to Store Room
            #Object3
            # (15.0, 3.9, -0.623, 0.781), #18   Research lab to store room Intermediate
            # (26.065, -2.714, -0.891, 0.454),  #19  Store Pickup 1
            # (26.065, -2.714, -0.994, 0.0),  #20   Store Pickup 1 Orientation to Exit
            # (25.8179, -3.3344, -0.871, 0.490), #21   Store Pickup 2
            # (25.8179, -3.3344, 0.894, -0.448), #22    Store Pickup 2 Orientation to Exit

            # (16.7, 1.0, 1, 0.0), #23   Store Pickup 2 out
            # ( 5.156, 0.861, -0.706, 0.7082), #24   Conference Intermediate CV
            # ( 5.070, -0.7, -0.9126, 0.4087), #25  Conference DropBox CV
            # ( 5.070, -0.7, -0.7583, -0.6518), #26  Conference Intermediate out CV
            # ( 5.18, 0.1, -0.7583, -0.6518), #27   Conference Intermediate out CV 1
            # ( 5.25, 0.65, -0.7583, -0.6518), #28  Conference Intermediate out CV 2
            #
            # (0,0,-0.719, -0.694)]

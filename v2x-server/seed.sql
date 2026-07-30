DELETE FROM road_segment_stats;
INSERT INTO road_segment_stats
  (edge_id, source, p1_lat, p1_lng, p2_lat, p2_lng, event_count, avg_risk, grade, avg_ttc)
VALUES
  ('e1','live',37.4958,126.9562,37.4965,126.9571,12,2.8,3,0.9),
  ('e2','live',37.4965,126.9571,37.4972,126.9580, 7,1.9,2,2.4),
  ('e3','live',37.4972,126.9580,37.4980,126.9575, 3,0.9,1,4.1),
  ('e1','sumo',37.4958,126.9562,37.4965,126.9571,40,2.1,2,1.8),
  ('e2','sumo',37.4965,126.9571,37.4972,126.9580,25,1.2,1,3.5),
  ('e4','sumo',37.4950,126.9558,37.4958,126.9566,30,1.7,2,2.9);

DELETE FROM zones;
INSERT INTO zones (zone_id, zone_name, zone_type, center_lat, center_lng, radius_m, base_risk)
VALUES
  ('Z01','정문 앞',   'crosswalk', 37.4966, 126.9573, 30, 3),
  ('Z02','주차장 출구','parking',   37.4958, 126.9562, 25, 4),
  ('Z03','중문',      'crosswalk', 37.4972, 126.9558, 30, 2);

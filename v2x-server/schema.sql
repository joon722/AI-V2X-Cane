CREATE TABLE IF NOT EXISTS events (
  id BIGSERIAL PRIMARY KEY,
  event_uid TEXT NOT NULL UNIQUE,
  source TEXT NOT NULL,
  device_id TEXT,
  scenario_id TEXT,
  lat DOUBLE PRECISION NOT NULL,
  lng DOUBLE PRECISION NOT NULL,
  edge_id TEXT,
  risk SMALLINT NOT NULL CHECK (risk BETWEEN 0 AND 3),
  ttc DOUBLE PRECISION,
  distance_m DOUBLE PRECISION,
  zone_id TEXT,
  occurred_at TIMESTAMPTZ NOT NULL,
  created_at TIMESTAMPTZ NOT NULL DEFAULT now()
);
CREATE INDEX IF NOT EXISTS ix_ev_src_time ON events (source, occurred_at);
CREATE INDEX IF NOT EXISTS ix_ev_edge_src ON events (edge_id, source);

CREATE TABLE IF NOT EXISTS road_segment_stats (
  edge_id TEXT NOT NULL,
  source TEXT NOT NULL,
  p1_lat DOUBLE PRECISION, p1_lng DOUBLE PRECISION,
  p2_lat DOUBLE PRECISION, p2_lng DOUBLE PRECISION,
  event_count INTEGER NOT NULL,
  avg_risk DOUBLE PRECISION NOT NULL,
  grade SMALLINT NOT NULL,
  avg_ttc DOUBLE PRECISION,
  agreement TEXT,
  updated_at TIMESTAMPTZ NOT NULL DEFAULT now(),
  PRIMARY KEY (edge_id, source)
);

CREATE TABLE IF NOT EXISTS zones (
  zone_id TEXT PRIMARY KEY,
  zone_name TEXT, zone_type TEXT,
  center_lat DOUBLE PRECISION,
  center_lng DOUBLE PRECISION,
  radius_m DOUBLE PRECISION,
  base_risk SMALLINT
);

CREATE TABLE IF NOT EXISTS ingest_log (
  id BIGSERIAL PRIMARY KEY,
  gcs_uri TEXT NOT NULL UNIQUE,
  source TEXT NOT NULL,
  scenario_id TEXT,
  row_count INTEGER NOT NULL,
  inserted_count INTEGER NOT NULL,
  skipped_count INTEGER NOT NULL,
  status TEXT NOT NULL,
  error TEXT,
  started_at TIMESTAMPTZ NOT NULL,
  finished_at TIMESTAMPTZ
);

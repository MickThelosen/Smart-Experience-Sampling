export interface ClickBeacon {
    beacon: Beacon;
    distance: number;
  }

export interface Beacon {
    uid: number;
}

export interface MapBeacon {
  x: number;
  y: number;
  uid: string;
  id: string | null;
}
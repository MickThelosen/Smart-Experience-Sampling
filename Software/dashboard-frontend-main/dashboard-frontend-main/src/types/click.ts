import { ClickBeacon } from "./beacon";
  
export interface Click {
    click_beacons: ClickBeacon[];
    click_time: string;
    clicker: {
        uid: number;
    };
    id: string;
}
  
  
export interface Location {
    id: string;
    name: string;
    parent_id: string;
    parent_location: Location
    child_locations: Location[]
}
  
  
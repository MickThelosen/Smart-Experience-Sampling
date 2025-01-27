import { Button } from "@/components/ui/button";
import { DialogFooter, DialogHeader, Dialog, DialogContent, DialogTitle } from "@/components/ui/dialog";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { Location } from "@/types/location";
import { ChevronRight, Upload } from "lucide-react";
import { useState } from "react";

interface CreateLocationModalProps {
    isOpen: boolean
    onClose: () => void
    onCreateLocation: (name: string, parentId: string | null, image?: File) => void
    locations: Location[]
  }

  
export function flattenLocations(locations: Location[], depth = 0): (Location & { depth: number })[] {
    return locations?.reduce((acc, location) => {
      return [
        ...acc,
        { ...location, depth },
        ...flattenLocations(location.child_locations, depth + 1)
      ]
    }, [] as (Location & { depth: number })[])
  }
  
export default function CreateLocationModal({ isOpen, onClose, onCreateLocation, locations }: CreateLocationModalProps) {
    const [name, setName] = useState('')
    const [parentId, setParentId] = useState<string | null>(null)
    const [image, setImage] = useState<File | null>(null)
  
    const flattenedLocations = flattenLocations(locations)
    
    const handleSubmit = (e: React.FormEvent) => {
        e.preventDefault()
        onCreateLocation(name, parentId === "none" ? null : parentId, image || undefined)
        setName('')
        setParentId(null)
        setImage(null)
        onClose()
      }
    
      const handleImageChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        if (e.target.files && e.target.files[0]) {
          setImage(e.target.files[0])
        }
      }

    return(
        <Dialog open={isOpen} onOpenChange={onClose}>
      <DialogContent className="sm:max-w-[425px]">
        <DialogHeader>
          <DialogTitle>Create New Location</DialogTitle>
        </DialogHeader>
        <form onSubmit={handleSubmit}>
          <div className="grid gap-4 py-4">
            <div className="grid grid-cols-4 items-center gap-4">
              <Label htmlFor="name" className="text-right">
                Name
              </Label>
              <Input
                id="name"
                value={name}
                onChange={(e) => setName(e.target.value)}
                className="col-span-3"
              />
            </div>
            <div className="grid grid-cols-4 items-center gap-4">
              <Label htmlFor="parent" className="text-right">
                Parent
              </Label>
              <Select onValueChange={(value) => setParentId(value === "none" ? null : value)} value={parentId || "none"}>
                <SelectTrigger className="col-span-3">
                  <SelectValue placeholder="Select a parent location (optional)" />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="none">No parent</SelectItem>
                  {flattenedLocations?.map((location) => (
                    <SelectItem key={location.id} value={location.id} className="flex items-center">
                      <div style={{ marginLeft: `${location.depth * 16}px` }} className="flex items-center">
                        {location.name}
                      </div>
                    </SelectItem>
                  ))}
                </SelectContent>
              </Select>
            </div>
            <div className="grid grid-cols-4 items-center gap-4">
              <Label htmlFor="image" className="text-right">
                Image
              </Label>
              <div className="col-span-3">
                <Input
                  id="image"
                  type="file"
                  onChange={handleImageChange}
                  accept="image/*"
                  className="hidden"
                />
                <Label htmlFor="image" className="cursor-pointer flex items-center justify-center w-full px-4 py-2 text-sm font-medium text-gray-700 bg-white border border-gray-300 rounded-md shadow-sm hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-indigo-500">
                  <Upload className="w-5 h-5 mr-2" />
                  {image ? image.name : "Upload Image"}
                </Label>
              </div>
            </div>
          </div>
          <DialogFooter>
            <Button type="submit">Create Location</Button>
          </DialogFooter>
        </form>
      </DialogContent>
    </Dialog>
    )
}
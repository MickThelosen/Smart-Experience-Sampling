import { useState } from "react";
import { Button } from "../ui/button";
import { Dialog, DialogContent, DialogFooter, DialogHeader, DialogTitle } from "../ui/dialog";
import { Input } from "../ui/input";
import { Label } from "../ui/label";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "../ui/select";

interface NewMapDialogProps {
    isOpen: boolean
    onClose: () => void
    onCreate: (name: string) => void
  }

export default function NewMapDialog({isOpen, onClose, onCreate} : NewMapDialogProps) {
    const [name, setName] = useState('')
    

    const handleSubmit = (e: React.FormEvent) => {
        e.preventDefault()
        onCreate(name)
        setName('')
        onClose()
      }

    return (<Dialog open={isOpen} onOpenChange={onClose}>
        <DialogContent className="sm:max-w-[425px]">
          <DialogHeader>
            <DialogTitle>Create new Beacon</DialogTitle>
          </DialogHeader>
          <form onSubmit={handleSubmit}>
            <div className="grid gap-4 py-4">
              <div className="grid grid-cols-4 items-center gap-4">
                <Label htmlFor="name" className="text-right">
                  Beacon UID
                </Label>
                <Input
                  id="name"
                  value={name}
                  onChange={(e) => setName(e.target.value)}
                  className="col-span-3"
                />
              </div>
            </div>
            <DialogFooter>
              <Button type="submit">Create Beacon</Button>
            </DialogFooter>
          </form>
        </DialogContent>
      </Dialog>)
}
import { LucideProps } from "lucide-react";
import { Button } from "./ui/button";
import { CSSProperties, ForwardRefExoticComponent, RefAttributes } from "react";

interface FABProperties {
    variant?: "link" | "default" | "destructive" | "outline" | "secondary" | "ghost" | null | undefined
    size?: "default" | "sm" | "lg" | "icon" | null | undefined
    icon: ForwardRefExoticComponent<Omit<LucideProps, "ref"> & RefAttributes<SVGSVGElement>>
    onClick?: (event: any) => void
}

export default function FAB(props : FABProperties) {
    return (
        <div style={{
            position: 'absolute',
            right: '20px',
            bottom: '20px'
        }}>
        <Button variant={props.variant} size={props.size} onClick={props.onClick}>
            <props.icon />
        </Button>
        </div>
    )
}
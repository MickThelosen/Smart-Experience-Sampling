import {
    Sidebar,
    SidebarContent,
    SidebarFooter,
    SidebarGroup,
    SidebarHeader,
    SidebarMenu,
    SidebarMenuButton,
    SidebarMenuItem,
  } from "@/components/ui/sidebar"
import { Calendar, Home, Inbox, Search, Settings, Microscope, Building2, User } from "lucide-react"
   
  const items = [
    {
      title: "Home",
      url: "/",
      icon: Home,
    },
    {
        title: "Researches",
        url: '/research',
        icon: Microscope
    },
    {
        title: "Locations",
        url: '/locations',
        icon: Building2
    },
    {
        title: "Users",
        url: '/users',
        icon: User
    },
    {
      title: "Settings",
      url: "/settings",
      icon: Settings,
    },
  ]

  export function AppSidebar() {
    return (
      <Sidebar>
        <SidebarContent>
          <SidebarMenu style={{
        padding: '20px'
      }}>
          {items.map((item) => (
                <SidebarMenuItem key={item.title}>
                  <SidebarMenuButton asChild>
                    <a href={item.url}>
                      <item.icon />
                      <span>{item.title}</span>
                    </a>
                  </SidebarMenuButton>
                </SidebarMenuItem>
              ))}
              </SidebarMenu>
        </SidebarContent>
      </Sidebar>
    )
  }
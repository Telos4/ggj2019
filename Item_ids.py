Roc_Para_id = 0
Roc_Cont_id = 256
Roc_Stor_id = 512
Roc_Fuel_id = 768
Roc_Thru_id = 64

Solar_Pan_id = 384
Battery_id = 576

Landscape_id = 640
Mission_id  = 320

Home_id = 832

Memory_01_id = 128

type_list = [
    [Roc_Cont_id, "item"],
    [Roc_Para_id, "item"],
    [Roc_Stor_id, "item"],
    [Roc_Fuel_id, "item"],
    [Roc_Thru_id,"item"],
    [Solar_Pan_id, "item"],
    [Battery_id, "item"],
    [Landscape_id, "memory"],
    [Mission_id, "item"],
    [Memory_01_id, "memory"]
]


item_dict = {
    "Roc_Parachute" : Roc_Para_id,
    "Roc_Control":Roc_Cont_id,
    "Roc_Storage":Roc_Stor_id,
    "Roc_Fuel":Roc_Fuel_id,
    "Roc_Thruster":Roc_Thru_id,
    "Solar_Panel": Solar_Pan_id,
    "Battery": Battery_id,
    "Mission":Mission_id,
}
pic_dict = {
    Roc_Cont_id: "Art/Raketenteile_Steuerelement.png",
    Roc_Para_id : "Art/Item_fallschirm.png",
    Roc_Stor_id : "Art/Raketenteile_Kapsel.png",
    Roc_Fuel_id : "Art/Treibstoff.png",
    Roc_Thru_id : "Art/Plan-_Triebwerk.png",
    Solar_Pan_id: "Art/Item_solarpanel.png",
    Battery_id: "Art/Battery.png",
    Landscape_id: ["Art/landscape.png","Art/Erinnerung1.png"],
    Mission_id : "Art/Rocket_Plan.png",
    Memory_01_id : ["Art/Erinnerung0.png","Art/Erinnerung1.png"]
}

time_dict = {
    Memory_01_id : 3,
    Landscape_id : 3
}




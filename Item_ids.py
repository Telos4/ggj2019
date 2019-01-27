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
# TODO implement correct ids
Memory_02_id = None
Memory_03_id = None
Memory_04_id = None
Memory_05_id = None

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
    Roc_Cont_id: "Raketenteile_Steuerelement.png",
    Roc_Para_id : "Item_fallschirm.png",
    Roc_Stor_id : "Raketenteile_Kapsel.png",
    Roc_Fuel_id : "Treibstoff.png",
    Roc_Thru_id : "Plan-_Triebwerk.png",
    Solar_Pan_id: "Item_solarpanel.png",
    Battery_id: "Battery.png",
    Landscape_id: ["landscape.png","Erinnerung1.png"],
    Mission_id : "Rocket_Plan.png",
    Memory_01_id : ["Erinnerung0.png","Erinnerung1.png"]
}

music_dict = {
    Memory_01_id: "Music/Erinnerung1.ogg",
    Memory_02_id: "Music/Erinnerung2.ogg",
    Memory_03_id: "Music/Erinnerung3.ogg",
    Memory_04_id: "Music/Erinnerung4.ogg",
    Memory_05_id: "Music/Erinnerung5.ogg"
}

music_transition_dict = {
    Memory_01_id: 5.5,
    Memory_02_id: 4.3,
    Memory_03_id: 8.0,
    Memory_04_id: 5.5,
    Memory_05_id: 3.0,
}

time_dict = {
    Memory_01_id : 3,
    Landscape_id : 3
}




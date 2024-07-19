#include "CANopen.h"
#include "PinNames.h"



class Master{

    public:

        Master(/*CO_OD_entry_t* od,*/uint16_t bitrate, uint8_t node_id); //PinName can_tx, PinName can_rx); 

        ~Master(){CO_delete((void *)(0x40000000UL + 0x6400UL));}         //Distruttore

        ///////////////////////////////////////////////////////////////////
        ///////////////////////     NMT FUNCTIONS   ///////////////////////
        ///////////////////////////////////////////////////////////////////
        
        void NMT_Enter_pre_operational(uint8_t slave_id);                    //Imposta un nodo in stato Pre-Operational

        void NMT_Reset_communication(uint8_t slave_id);                      //Reset della componente adibita alla comunicazione

        void NMT_Reset_node(uint8_t slave_id);                               //Reset di un nodo

        void NMT_Set_Operational_State();                                    //Imposta il master in stato Operational

        void NMT_Start_remote_node(uint8_t slave_id);                        //Imposta un nodo in stato Operational

        void NMT_Stop_remote_node(uint8_t slave_id);                         //Disattiva l'attività di un nodo      
        
        ///////////////////////     NMT CALLBACKS   ///////////////////////
        
        
        
        ///////////////////////////////////////////////////////////////////
        ///////////////////////     SYNC FUNCTIONS   //////////////////////
        ///////////////////////////////////////////////////////////////////

        bool SYNC_Start_SYNC_protocol(); 

        bool SYNC_Stop_SYNC_protocol(); //usando SDO l'utente scrive zero nella voce 1006 dell'OD

        bool SYNC_Set_counter(uint8_t value); 
        //usando SDO l'utente scrive "value" nella voce 1019
        // ritorna "true" se l'operazione ha successo, "false" se fallisce o value è fuori range  (2:240)

        bool SYNC_Set_Communication_cycle_period(uint32_t Communication_cycle_period_ms); 
        //usando SDO l'utente modifica la finestra temporale di invio (voce 1006 dell' OD)

        ///////////////////////////////////////////////////////////////////
        ///////////////////////     EMCY FUNCTIONS   //////////////////////
        ///////////////////////////////////////////////////////////////////
        
        //get_log -> resituisce un elenco di codici errore se presenti


        ///////////////////////////////////////////////////////////////////
        ///////////////////////     TIME FUNCTIONS   //////////////////////
        ///////////////////////////////////////////////////////////////////

        void TIME_Set_global_network_time();

        ///////////////////////////////////////////////////////////////////
        ///////////////////////     PDO FUNCTIONS   ///////////////////////
        ///////////////////////////////////////////////////////////////////
        
        ///////////////////////////////////////////////////////////////////
        ///////////////////////     SDO FUNCTIONS   ///////////////////////
        ///////////////////////////////////////////////////////////////////

        void SDO_Download_SDO(uint8_t slave_id, uint16_t index, uint8_t subIndex, uint8_t *dataTx, uint32_t dataSize); //lato client

        void SDO_Upload_SDO(uint8_t slave_id, uint16_t index, uint8_t subIndex, uint8_t *dataRx, uint32_t dataSize, uint32_t* data_size, uint32_t* abort);   //lato client

        void SDO_Start_process_SDO(uint16_t Timeout_timer); //lato server

        ///////////////////////////////////////////////////////////////////
        ///////////////////////     HB FUNCTIONS   ////////////////////////
        ///////////////////////////////////////////////////////////////////

        void HB_Start_process_HB();
        
        void HB_Start_produce_HB();

        void HB_Set_slave_hb_producer_time(uint8_t slave_id,uint8_t index, uint16_t hb_producer_time_ms,uint16_t hb_consumer_time_ms);

        bool HB_Set_producer_hb_time(uint16_t time_ms);

        //void HB_Set_hb_callback(uint8_t node_id, void (*Function_to_call));

        ///////////////////////     HB CALLBACKS   ///////////////////////

        //void Set_Bootup_callback(uint8_t node_id, void (*Function_to_call)(uint8_t,uint8_t,void*));  //Impostare le azioni da compiere quando il master riceve messaggi di bootup dai nodi monitorati
        
        
        //const PinName get_txpinname()const {return Can_Tx;}
        //const PinName get_rxpinname()const {return Can_Rx;}
        const uint16_t get_bitrate() const {return Bitrate;}
        const uint8_t get_nodeid() const {return Node_Id;}

    private:

        uint16_t Bitrate;
        //PinName Can_Tx;
        //PinName Can_Rx;
        uint8_t Node_Id;
        //CO_OD_entry_t* Object_Dictonary;



};
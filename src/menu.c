#include "menu.h"
#include "main.h"
#include "buttons.h"
#include "dcts.h"
#include "dcts_config.h"
#include "string.h"

/**
  * @defgroup menu
  * @brief work with menu
  */


static const menuItem Null_Menu = {
    .Next = (void*)0,
    .Previous = (void*)0,
    .Parent = (void*)0,
    .Child = (void*)0,
    .Child_num = 0,
    .Page = 0,
    .Text = {0},
};
const menuItem edit_value = {
    .Next = (void*)0,
    .Previous = (void*)0,
    .Parent = (void*)0,
    .Child = (void*)0,
    .Child_num = 0,
    .Page = EDIT,
    .Text = {0},
};

menuItem* selectedMenuItem;

//                  NAME           NEXT            PREV            PARENT          CHILD        GHILD_NUM   PAGE                    TEXT
MAKE_MENU       (main_page,     NULL_ENTRY,     NULL_ENTRY,     NULL_ENTRY,     common_info,    4,          MAIN_PAGE,          "������� ����");
  MAKE_MENU     (common_info,   meas_channels,  date,           main_page,      info,           1,          COMMON_INFO,        "�� ����������");
    MAKE_MENU   (info,          NULL_ENTRY,     NULL_ENTRY,     common_info,    NULL_ENTRY,     0,          INFO,               "�� ����������");
  MAKE_MENU     (meas_channels, act_channels,   common_info,    main_page,      meas_ch_0,      10,         MEAS_CHANNELS,      "���. ������");
    MAKE_MENU   (meas_ch_0,     meas_ch_1,      meas_ch_9,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_0,          "����. ����");
    MAKE_MENU   (meas_ch_1,     meas_ch_2,      meas_ch_0,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_1,          "����. ���� ��");
    MAKE_MENU   (meas_ch_2,     meas_ch_3,      meas_ch_1,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_2,          "����. ���� ���");
    MAKE_MENU   (meas_ch_3,     meas_ch_4,      meas_ch_2,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_3,          "����. ���� �");
    MAKE_MENU   (meas_ch_4,     meas_ch_5,      meas_ch_3,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_4,          "����. �����");
    MAKE_MENU   (meas_ch_5,     meas_ch_6,      meas_ch_4,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_5,          "����. ����� ���");
    MAKE_MENU   (meas_ch_6,     meas_ch_7,      meas_ch_5,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_6,          "����. ����� �");
    MAKE_MENU   (meas_ch_7,     meas_ch_8,      meas_ch_6,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_7,          "������� ����. �");
    MAKE_MENU   (meas_ch_8,     meas_ch_9,      meas_ch_7,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_8,          "��������� �");
    MAKE_MENU   (meas_ch_9,     meas_ch_0,      meas_ch_8,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_9,          "�����������");
  MAKE_MENU     (act_channels,  rele_channels,  meas_channels,  main_page,      act_ch_0,       2,          ACT_CHANNELS,       "���. ������");
    MAKE_MENU   (act_ch_0,      act_ch_1,       act_ch_1,       act_channels,   act_en_0,       4,          ACT_CH_0,           "����������� ����");
      MAKE_MENU (act_en_0,      act_set_0,      act_cur_0,      act_ch_0,       EDITED_VAL,     0,          ACT_EN_0,           "����������");
      MAKE_MENU (act_set_0,     act_hyst_0,     act_en_0,       act_ch_0,       EDITED_VAL,     0,          ACT_SET_0,          "������");
      MAKE_MENU (act_hyst_0,    act_cur_0,      act_set_0,      act_ch_0,       EDITED_VAL,     0,          ACT_HYST_0,         "����������");
      MAKE_MENU (act_cur_0,     act_en_0,       act_hyst_0,     act_ch_0,       NULL_ENTRY,     0,          ACT_CUR_0,          "�������");
    MAKE_MENU   (act_ch_1,      act_ch_0,       act_ch_0,       act_channels,   act_en_1,       4,          ACT_CH_0,           "����������� �����");
      MAKE_MENU (act_en_1,      act_set_1,      act_cur_1,      act_ch_1,       EDITED_VAL,     0,          ACT_EN_0,           "����������");
      MAKE_MENU (act_set_1,     act_hyst_1,     act_en_1,       act_ch_1,       EDITED_VAL,     0,          ACT_SET_0,          "������");
      MAKE_MENU (act_hyst_1,    act_cur_1,      act_set_1,      act_ch_1,       EDITED_VAL,     0,          ACT_HYST_0,         "����������");
      MAKE_MENU (act_cur_1,     act_en_1,       act_hyst_1,     act_ch_1,       NULL_ENTRY,     0,          ACT_CUR_0,          "�������");
  MAKE_MENU     (rele_channels, display,        act_channels,   main_page,      rele_ch_0,      1,          RELE_CHANNELS,      "�������� ������");
    MAKE_MENU   (rele_ch_0,     rele_ch_0,      rele_ch_0,      rele_channels,  rele_auto_0,    2,          RELE_CH_0,          "�����������");
      MAKE_MENU (rele_auto_0,   rele_cntrl_0,   rele_cntrl_0,   rele_ch_0,      EDITED_VAL,     0,          RELE_AUTO_MAN_0,    "����������");
      MAKE_MENU (rele_cntrl_0,  rele_auto_0,    rele_auto_0,    rele_ch_0,      EDITED_VAL,     0,          RELE_CONTROL_0,     "���������");
  MAKE_MENU     (display,       time,           rele_channels,  main_page,      light_lvl,      2,          DISPLAY,            "�������");
    MAKE_MENU   (light_lvl,     auto_off,       auto_off,       display,        EDITED_VAL,     0,          LIGHT_LVL,          "�������");
    MAKE_MENU   (auto_off,      light_lvl,      light_lvl,      display,        EDITED_VAL,     0,          AUTO_OFF,           "��������. ���������");
  MAKE_MENU     (time,          date,           display,        main_page,      time_hour,      3,          TIME,               "�����");
    MAKE_MENU   (time_hour,     time_min,       time_sec,       time,           EDITED_VAL,     0,          TIME_HOUR,          "����");
    MAKE_MENU   (time_min,      time_sec,       time_hour,      time,           EDITED_VAL,     0,          TIME_MIN,           "������");
    MAKE_MENU   (time_sec,      time_hour,      time_min,       time,           EDITED_VAL,     0,          TIME_SEC,           "�������");
  MAKE_MENU     (date,          common_info,    time,           main_page,      date_day,       3,          DATE,               "����");
    MAKE_MENU   (date_day,      date_month,     date_year,      date,           EDITED_VAL,     0,          DATE_DAY,           "����");
    MAKE_MENU   (date_month,    date_year,      date_day,       date,           EDITED_VAL,     0,          DATE_MONTH,         "�����");
    MAKE_MENU   (date_year,     date_day,       date_month,     date,           EDITED_VAL,     0,          DATE_YEAR,          "���");

MAKE_MENU       (save_changes,  NULL_ENTRY,     NULL_ENTRY,     NULL_ENTRY,     NULL_ENTRY,     0,          SAVE_CHANGES,       "��������� ���.");


/*========== FUNCTIONS ==========*/

void menu_init (void){
    selectedMenuItem = &main_page;
}

void menuChange(menuItem* NewMenu){
    if ((NewMenu != &NULL_ENTRY)&&(NewMenu != &EDITED_VAL)){
        selectedMenuItem = NewMenu;
    }else if(NewMenu == &EDITED_VAL){
        navigation_style = DIGIT_EDIT;
    }
}

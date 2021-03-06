PULP_PROPERTIES += udma/cpi/version pulp_chip_family board/name

include $(TARGET_INSTALL_DIR)/rules/pulp_properties.mk

INSTALL_FILES += $(shell find include -name *.h)

$(info BOARD $(board/name))

ifneq '$(board/name)' ''
build_dir_ext=_$(board/name)
ifeq '$(board/name)' 'gapoc_a'
PULP_LIBS += pibsp_gapoc_a
endif
ifeq '$(board/name)' 'gapuino'
PULP_LIBS += pibsp_gapuino
endif
ifeq '$(board/name)' 'ai_deck'
PULP_LIBS += pibsp_ai_deck
endif
else
ifeq '$(pulp_chip_family)' 'gap'
PULP_LIBS += pibsp_gapoc_a pibsp_gapuino pibsp_ai_deck
endif
endif

ifeq '$(pulp_chip_family)' 'wolfe'
PULP_LIBS += pibsp_wolfe
endif

ifeq '$(pulp_chip_family)' 'vega'
PULP_LIBS += pibsp_vega
endif

ifeq '$(pulp_chip_family)' 'gap9'
PULP_LIBS += pibsp_gap9
endif

PULP_LIB_FC_SRCS_pibsp_gap9 = $(VEGA_SRC)
PULP_LIB_TARGET_NAME_pibsp_gap9 = gap9/libpibsp.a
PULP_LIB_CFLAGS_pibsp_gap9 = -DCONFIG_GAP9

PULP_LIB_FC_SRCS_pibsp_vega = $(VEGA_SRC)
PULP_LIB_TARGET_NAME_pibsp_vega = vega/libpibsp.a
PULP_LIB_CFLAGS_pibsp_vega = -DCONFIG_VEGA

PULP_LIB_FC_SRCS_pibsp_wolfe = $(WOLFE_SRC)
PULP_LIB_TARGET_NAME_pibsp_wolfe = wolfe/libpibsp.a
PULP_LIB_CFLAGS_pibsp_wolfe = -DCONFIG_WOLFE

PULP_LIB_FC_SRCS_pibsp_gapoc_a = $(GAPOC_A_SRC)
PULP_LIB_TARGET_NAME_pibsp_gapoc_a = gapoc_a/libpibsp.a
PULP_LIB_CFLAGS_pibsp_gapoc_a = -DCONFIG_GAPOC_A

PULP_LIB_FC_SRCS_pibsp_gapuino = $(GAPUINO_SRC)
PULP_LIB_TARGET_NAME_pibsp_gapuino = gapuino/libpibsp.a
PULP_LIB_CFLAGS_pibsp_gapuino = -DCONFIG_GAPUINO

PULP_LIB_FC_SRCS_pibsp_ai_deck = $(AI_DECK_SRC)
PULP_LIB_TARGET_NAME_pibsp_ai_deck = ai_deck/libpibsp.a
PULP_LIB_CFLAGS_pibsp_ai_deck = -DCONFIG_AI_DECK

PULP_CFLAGS += -Os -g -I$(CURDIR)/include
PULP_CFLAGS += -Wextra -Wall -Werror -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function -Wundef

include $(PULP_SDK_HOME)/install/rules/pulp.mk

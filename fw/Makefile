
BINARY = main
OBJS += libuxb.o solar_charger_iface.pb.o uxb_slave.o nanopb/pb_common.o nanopb/pb_encode.o battery_monitor.o voltage_control.o ntc.o

CFLAGS += -Inanopb
LDSCRIPT = stm32f042g6u6.ld
LDFLAGS += --specs=rdimon.specs
LDFLAGS += --specs=nano.specs
LDLIBS	+= -lrdimon -lm

include Makefile.include

proto:
	protoc --plugin=nanopb/generator/protoc-gen-nanopb --proto_path=. --nanopb_out=. --proto_path=nanopb/generator/proto *.proto

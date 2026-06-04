GCC     = riscv64-unknown-elf-gcc
OBJCOPY = riscv64-unknown-elf-objcopy
IVERILOG= iverilog
VVP     = vvp
GTKWAVE = gtkwave

DIR_RTL = ./rtl
DIR_SW  = ./sw
DIR_SIM = ./sim

FW_SRC  = $(DIR_SW)/start.s $(DIR_SW)/main.c
FW_LD   = $(DIR_SW)/linker.ld
FW_ELF  = $(DIR_SW)/firmware.elf
FW_BIN  = $(DIR_SW)/firmware.bin
FW_HEX  = $(DIR_SW)/firmware.hex

RTL_SRC = $(DIR_RTL)/soc_top.v \
          $(DIR_RTL)/core/my_riscv_core.v \
          $(DIR_RTL)/core/alu.v \
          $(DIR_RTL)/core/reg_file.v \
          $(DIR_RTL)/core/decoder.v \
          $(DIR_RTL)/core/control_unit.v \
          $(DIR_RTL)/bus/bus_arbiter.v \
          $(DIR_RTL)/dma/dma_controller.v

TB_SRC  = $(DIR_SIM)/tb_soc.v
SIM_OUT = $(DIR_SIM)/soc_sim
WAVE_OUT= $(DIR_SIM)/soc_wave.vcd


all: sim

fw: $(FW_HEX)

$(FW_HEX): $(FW_SRC)
	$(GCC) -march=rv32i -mabi=ilp32 -O2 -ffreestanding -nostdlib -T $(FW_LD) $(FW_SRC) -o $(FW_ELF)
	$(OBJCOPY) -O binary $(FW_ELF) $(FW_BIN)
	python3 -c "import struct; f=open('$(FW_BIN)','rb'); d=f.read(); f.close(); [print('{:08x}'.format(struct.unpack('<I', d[i:i+4])[0])) for i in range(0, len(d), 4)]" > $(FW_HEX)

sim: fw
	$(IVERILOG) -o $(SIM_OUT) $(RTL_SRC) $(TB_SRC)
	cd $(DIR_SIM) && $(VVP) soc_sim

wave:
	cd $(DIR_SIM) && $(GTKWAVE) soc_wave.vcd &

clean:
	rm -f $(DIR_SW)/*.elf $(DIR_SW)/*.bin $(DIR_SW)/*.hex
	rm -f $(DIR_SIM)/soc_sim $(DIR_SIM)/*.vcd
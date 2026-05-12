# ==========================================
# MAKEFILE CHO RISC-V SOC PROJECT
# ==========================================

# --- 1. Khai báo công cụ (Toolchain) ---
GCC     = riscv64-unknown-elf-gcc
OBJCOPY = riscv64-unknown-elf-objcopy
IVERILOG= iverilog
VVP     = vvp
GTKWAVE = gtkwave

# --- 2. Khai báo thư mục (Directories) ---
DIR_RTL = ./rtl
DIR_SW  = ./sw
DIR_SIM = ./sim

# --- 3. Khai báo file phần mềm (Firmware) ---
FW_SRC  = $(DIR_SW)/start.s $(DIR_SW)/main.c
FW_LD   = $(DIR_SW)/linker.ld
FW_ELF  = $(DIR_SW)/firmware.elf
FW_BIN  = $(DIR_SW)/firmware.bin
FW_HEX  = $(DIR_SW)/firmware.hex

# --- 4. Khai báo file phần cứng (Hardware RTL) ---
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

# ==========================================
# CÁC LỆNH THỰC THI (TARGETS)
# ==========================================

# Lệnh mặc định khi chỉ gõ "make"
all: sim

# --- Lệnh 1: Dịch code C ra mã Hex (Firmware) ---
fw: $(FW_HEX)

$(FW_HEX): $(FW_SRC)
	@echo "--- DỊCH CODE C SANG MÃ MÁY RISC-V ---"
	$(GCC) -march=rv32i -mabi=ilp32 -O2 -ffreestanding -nostdlib -T $(FW_LD) $(FW_SRC) -o $(FW_ELF)
	$(OBJCOPY) -O binary $(FW_ELF) $(FW_BIN)
	python3 -c "import struct; f=open('$(FW_BIN)','rb'); d=f.read(); f.close(); [print('{:08x}'.format(struct.unpack('<I', d[i:i+4])[0])) for i in range(0, len(d), 4)]" > $(FW_HEX)
	@echo "=> Da tao xong firmware.hex!"

# --- Lệnh 2: Biên dịch và chạy mô phỏng Phần cứng ---
# Chú ý: Lệnh này phụ thuộc vào lệnh 'fw', tức là nó sẽ tự động dịch code C trước nếu code C bị thay đổi.
sim: fw
	@echo "--- BIÊN DỊCH PHẦN CỨNG VERILOG ---"
	$(IVERILOG) -o $(SIM_OUT) $(RTL_SRC) $(TB_SRC)
	@echo "--- CHẠY MÔ PHỎNG ---"
	cd $(DIR_SIM) && $(VVP) soc_sim

# --- Lệnh 3: Mở sóng GTKWave ---
wave:
	@echo "--- MỞ BẢN ĐỒ SÓNG GTKWAVE ---"
	cd $(DIR_SIM) && $(GTKWAVE) soc_wave.vcd &

# --- Lệnh 4: Dọn dẹp rác ---
clean:
	@echo "--- DỌN DẸP CÁC FILE TẠM ---"
	rm -f $(DIR_SW)/*.elf $(DIR_SW)/*.bin $(DIR_SW)/*.hex
	rm -f $(DIR_SIM)/soc_sim $(DIR_SIM)/*.vcd
	@echo "=> Da don dep sach se!"
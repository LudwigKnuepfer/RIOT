PKG_NAME=libcoap
PKG_URL=http://git.code.sf.net/p/libcoap/code
PKG_VERSION=ef41ce5d02d64cec0751882ae8fd95f6c32bc018
PKG_DIR=$(CURDIR)/$(PKG_NAME)

.PHONY: all clean patch reset

all: patch
	"$(MAKE)" -C $(PKG_DIR)

patch: $(PKG_DIR)/Makefile

$(PKG_DIR)/Makefile: $(PKG_DIR)/.git/config
	@cd "$(PKG_DIR)" && git am -q --ignore-whitespace "$(CURDIR)"/*.patch

$(PKG_DIR)/.git/config:
	@test -d "$(PKG_DIR)" || git clone "$(PKG_URL)" "$(PKG_DIR)"; \
		cd "$(PKG_DIR)" && git checkout -q -f "$(PKG_VERSION)"

clean: $(PKG_DIR)/Makefile
	@cd "$(PKG_DIR)" && \
		git clean -x -f && \
		git am --abort 2>/dev/null ; \
		git reset -q --hard "$(PKG_VERSION)"
	@"$(MAKE)" patch


distclean:
	@rm -rf "$(PKG_DIR)"

Makefile.include:
	@true

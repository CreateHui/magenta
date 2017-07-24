// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <hw/reg.h>
#include <stdio.h>

#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/protocol/gpio.h>
#include <ddk/protocol/platform-device.h>

#include "hi3660-bus.h"
#include "hi3660-regs.h"

#define IP_RST_USB3OTG_MUX                 (1 << 8)
#define IP_RST_USB3OTG_AHBIF                 (1 << 7)
#define IP_RST_USB3OTG_32K                 (1 << 6)
#define IP_RST_USB3OTG                     (1 << 5)
#define IP_RST_USB3OTGPHY_POR                 (1 << 3)

#define SC_USB3PHY_ABB_GT_EN            (1 << 15)

# define USBOTG3CTRL2_POWERDOWN_HSP             (1 << 0)
# define USBOTG3CTRL2_POWERDOWN_SSP             (1 << 1)

#define USB3OTG_ACLK_FREQ        229000000

static void phy_cr_wait_ack(volatile void *otg_bc_base)
{
    int i = 1000;

    while (1) {
        if ((readl(otg_bc_base + USB3OTG_PHY_CR_STS) & USB3OTG_PHY_CR_ACK) == 1)
            break;
        mx_nanosleep(mx_deadline_after(MX_USEC(50)));
        if (i-- < 0) {
            printf("wait phy_cr_ack timeout!\n");
            break;
        }
    }
}

static void phy_cr_set_addr(volatile void *otg_bc_base, uint32_t addr)
{
    uint32_t reg;

    /* set addr */
    reg = USB3OTG_PHY_CR_DATA_IN(addr);
    writel(reg, otg_bc_base + USB3OTG_PHY_CR_CTRL);

    mx_nanosleep(mx_deadline_after(MX_USEC(100)));

    /* cap addr */
    reg = readl(otg_bc_base + USB3OTG_PHY_CR_CTRL);
    reg |= USB3OTG_PHY_CR_CAP_ADDR;
    writel(reg, otg_bc_base + USB3OTG_PHY_CR_CTRL);

    phy_cr_wait_ack(otg_bc_base);

    /* clear ctrl reg */
    writel(0, otg_bc_base + USB3OTG_PHY_CR_CTRL);
}

static uint16_t phy_cr_read(volatile void *otg_bc_base, uint32_t addr)
{
    uint32_t reg;
    int i = 1000;

    phy_cr_set_addr(otg_bc_base, addr);

    /* read cap */
    writel(USB3OTG_PHY_CR_READ, otg_bc_base + USB3OTG_PHY_CR_CTRL);

    mx_nanosleep(mx_deadline_after(MX_USEC(100)));

    while (1) {
        reg = readl(otg_bc_base + USB3OTG_PHY_CR_STS);
        if ((reg & USB3OTG_PHY_CR_ACK) == 1) {
            break;
        }
    mx_nanosleep(mx_deadline_after(MX_USEC(50)));
        if (i-- < 0) {
            printf("wait phy_cr_ack timeout!\n");
            break;
        }
    }

    /* clear ctrl reg */
    writel(0, otg_bc_base + USB3OTG_PHY_CR_CTRL);

    return (uint16_t)USB3OTG_PHY_CR_DATA_OUT(reg);
}

static void phy_cr_write(volatile void *otg_bc_base, uint32_t addr, uint32_t value)
{
    uint32_t reg;

    phy_cr_set_addr(otg_bc_base, addr);

    reg = USB3OTG_PHY_CR_DATA_IN(value);
    writel(reg, otg_bc_base + USB3OTG_PHY_CR_CTRL);

    /* cap data */
    reg = readl(otg_bc_base + USB3OTG_PHY_CR_CTRL);
    reg |= USB3OTG_PHY_CR_CAP_DATA;
    writel(reg, otg_bc_base + USB3OTG_PHY_CR_CTRL);

    /* wait ack */
    phy_cr_wait_ack(otg_bc_base);

    /* clear ctrl reg */
    writel(0, otg_bc_base + USB3OTG_PHY_CR_CTRL);

    reg = USB3OTG_PHY_CR_WRITE;
    writel(reg, otg_bc_base + USB3OTG_PHY_CR_CTRL);

    /* wait ack */
    phy_cr_wait_ack(otg_bc_base);
}

#define DWC3_PHY_RX_OVRD_IN_HI    0x1006
#define DWC3_PHY_RX_SCOPE_VDCC    0x1026
#define RX_SCOPE_LFPS_EN    (1 << 0)
#define TX_VBOOST_LVL_MASK             7
#define TX_VBOOST_LVL(x)               ((x) & TX_VBOOST_LVL_MASK)

#define REF_SSP_EN                      (1 << 16)

static int usb3_clk_init(volatile void* pctrl_base, volatile void* pericfg_base)
{
//    int ret;
    uint32_t temp;

printf("DWC usb3_clk_init\n");

#if 0 // FIXME
#define HI3660_ACLK_GATE_USB3OTG    73
    // aclk_usb3otg <&crg_ctrl HI3660_ACLK_GATE_USB3OTG>;
    
    
    /* set usb aclk 240MHz to improve performance */
    ret = clk_set_rate(hisi_dwc3->gt_aclk_usb3otg, USB3OTG_ACLK_FREQ);
    if (ret)
        printf("usb aclk set rate failed\n");

    ret = clk_prepare_enable(hisi_dwc3->gt_aclk_usb3otg);
    if (ret) {
        printf("clk_prepare_enable gt_aclk_usb3otg failed\n");
        return ret;
    }
#endif

    /* usb refclk iso enable */
    writel(PERISOEN_USB_REFCLK_ISO_EN, pericfg_base + PERI_CRG_ISODIS);

    /* enable usb_tcxo_en */
    writel(USB_TCXO_EN | (USB_TCXO_EN << PERI_CTRL3_MSK_START),
           pctrl_base + PCTRL_CTRL3);

    /* select usbphy clk from abb */
    temp = readl(pctrl_base + PCTRL_CTRL24);
    temp &= ~SC_CLK_USB3PHY_3MUX1_SEL;
    writel(temp, pctrl_base + PCTRL_CTRL24);

    /* open clk gate */
    writel(GT_CLK_USB3OTG_REF | GT_ACLK_USB3OTG,
           pericfg_base + PERI_CRG_CLK_EN4);

#if 0 // FIXME
#define HI3660_CLK_ABB_USB        27
// clk_usb3phy_ref <&crg_ctrl HI3660_CLK_ABB_USB>

    ret = clk_prepare_enable(hisi_dwc3->clk);
    if (ret) {
        printf("clk_prepare_enable clk failed\n");
        return ret;
    }
#endif

// ?????
    writel(PEREN4_GT_ACLK_USB3OTG | PEREN4_GT_CLK_USB3OTG_REF, pericfg_base + PERI_CRG_PEREN4);

    return 0;
}

static void dwc3_release(volatile void* pericfg_base, volatile void* otg_bc_base)
{
    uint32_t temp;

    /* dis-reset the module */
    writel(IP_RST_USB3OTG_MUX | IP_RST_USB3OTG_AHBIF | IP_RST_USB3OTG_32K,
           pericfg_base + PERI_CRG_RSTDIS4);

    /* reset phy */
    writel(IP_RST_USB3OTGPHY_POR | IP_RST_USB3OTG,
           pericfg_base + PERI_CRG_RSTEN4);

    /* enable phy ref clk */
    temp = readl(otg_bc_base + USB3OTG_CTRL0);
    temp |= SC_USB3PHY_ABB_GT_EN;
    writel(temp, otg_bc_base + USB3OTG_CTRL0);

    temp = readl(otg_bc_base + USB3OTG_CTRL7);
    temp |= REF_SSP_EN;
    writel(temp, otg_bc_base + USB3OTG_CTRL7);

    /* exit from IDDQ mode */
    temp = readl(otg_bc_base + USB3OTG_CTRL2);
    temp &= ~(USBOTG3CTRL2_POWERDOWN_HSP | USBOTG3CTRL2_POWERDOWN_SSP);
    writel(temp, otg_bc_base + USB3OTG_CTRL2);

    mx_nanosleep(mx_deadline_after(MX_USEC(100)));

    /* dis-reset phy */
    writel(IP_RST_USB3OTGPHY_POR, pericfg_base + PERI_CRG_RSTDIS4);

    /* dis-reset controller */
    writel(IP_RST_USB3OTG, pericfg_base + PERI_CRG_RSTDIS4);

    mx_nanosleep(mx_deadline_after(MX_MSEC(20)));

    /* fake vbus valid signal */
    temp = readl(otg_bc_base + USB3OTG_CTRL3);
    temp |= (USB3OTG_CTRL3_VBUSVLDEXT | USB3OTG_CTRL3_VBUSVLDEXTSEL);
    writel(temp, otg_bc_base + USB3OTG_CTRL3);

    mx_nanosleep(mx_deadline_after(MX_USEC(100)));
}


void config_femtophy_param(volatile void* otg_bc_base)
{
    uint32_t reg;

    /* set high speed phy parameter */
    if (0 /* host */ ) {
//        writel(hisi_dwc->eye_diagram_host_param, otg_bc_base + USB3OTG_CTRL4);
//        printf("set hs phy param 0x%x for host\n",
//                readl(otg_bc_base + USB3OTG_CTRL4));
    } else {
        writel(0x01c466e3, otg_bc_base + USB3OTG_CTRL4);
        printf("set hs phy param 0x%x for device\n",
                readl(otg_bc_base + USB3OTG_CTRL4));
    }

    /* set usb3 phy cr config for usb3.0 */

    if (0 /*hisi_dwc->host_flag*/) {
//        phy_cr_write(otg_bc_base, DWC3_PHY_RX_OVRD_IN_HI,
//                hisi_dwc->usb3_phy_host_cr_param);
    } else {
        phy_cr_write(otg_bc_base, DWC3_PHY_RX_OVRD_IN_HI,
                0xb80 /*hisi_dwc->usb3_phy_cr_param*/);
    }

    printf("set ss phy rx equalization 0x%x\n",
            phy_cr_read(otg_bc_base, DWC3_PHY_RX_OVRD_IN_HI));

    /* enable RX_SCOPE_LFPS_EN for usb3.0 */
    reg = phy_cr_read(otg_bc_base, DWC3_PHY_RX_SCOPE_VDCC);
    reg |= RX_SCOPE_LFPS_EN;
    phy_cr_write(otg_bc_base, DWC3_PHY_RX_SCOPE_VDCC, reg);

    printf("set ss RX_SCOPE_VDCC 0x%x\n",
            phy_cr_read(otg_bc_base, DWC3_PHY_RX_SCOPE_VDCC));

    reg = readl(otg_bc_base + USB3OTG_CTRL6);
    reg &= ~TX_VBOOST_LVL_MASK;
    reg |= TX_VBOOST_LVL(0x5 /*hisi_dwc->usb3_phy_tx_vboost_lvl*/);
    writel(reg, otg_bc_base + USB3OTG_CTRL6);
    printf("set ss phy tx vboost lvl 0x%x\n", readl(otg_bc_base + USB3OTG_CTRL6));
}

mx_status_t hi3360_usb_init(hi3660_bus_t* bus) {
printf("hi3360_usb_init\n");
    volatile void* usb3otg_bc = bus->usb3otg_bc.vaddr;
    volatile void* peri_crg = bus->peri_crg.vaddr;
    volatile void* pctrl = bus->pctrl.vaddr;

    gpio_protocol_t gpio;
    if (pdev_get_protocol(&bus->pdev, MX_PROTOCOL_GPIO, &gpio) != MX_OK) {
        printf("hi3360_usb_init: could not get GPIO protocol!\n");
        return MX_ERR_INTERNAL;
    }

    // disable host vbus
    gpio_config(&gpio, 46, GPIO_DIR_OUT);
    gpio_write(&gpio, 46, 0);
    // enable type-c vbus
    gpio_config(&gpio, 202, GPIO_DIR_OUT);
    gpio_write(&gpio, 202, 1);


    usb3_clk_init(pctrl, peri_crg);
    dwc3_release(peri_crg, usb3otg_bc);
    config_femtophy_param(usb3otg_bc);

    return MX_OK;
}

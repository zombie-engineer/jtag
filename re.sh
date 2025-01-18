#!/bin/bash

echo 0000:05:00.3 > /sys/bus/pci/drivers/xhci_hcd/unbind
sleep 2
echo 0000:05:00.3 > /sys/bus/pci/drivers/xhci_hcd/bind

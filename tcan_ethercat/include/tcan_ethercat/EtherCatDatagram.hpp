/*
 * EtherCatDatagramData.hpp
 *
 *  Created on: Mar 24, 2017
 *      Author: Remo Diethelm
 */

#pragma once


// c++
#include <cassert>
#include <unordered_map>


namespace tcan_ethercat {


class EtherCatDatagram {
 public:
    EtherCatDatagram() {}
    virtual ~EtherCatDatagram() {}

    inline void resize(const uint16_t length) {
        uint8_t* oldData = data_;
        data_ = new uint8_t[length];
        std::copy(&oldData[0], &oldData[getDataLength()], data_);
        setDataLength(length);
        delete[] oldData;
    }

    template <typename T>
    inline void write(const uint16_t memoryPosition, const T& data) {
        assert(memoryPosition + sizeof(T) < getDataLength() && "Write memory out of range.");
        std::copy(&data_[memoryPosition], &data_[memoryPosition + sizeof(T)], &data);
    }

    inline const uint16_t getTotalLength() const { return getDataLength() + 12; }
    inline const uint16_t getDataLength() const { return header_.lenRCM_.elements_.len_; }
    inline const uint8_t* getData() const { return data_; }
    inline const uint16_t getWorkingCounter() const { return workingCounter_; }

    inline void setZero() {
        for (uint16_t i = 0; i < getDataLength(); i++) {
            data_[i] = 0;
        }
    }

 private:
    inline void setDataLength(uint16_t length) { header_.lenRCM_.elements_.len_ = length; }

    struct EtherCatDatagramHeader {
        enum class Command : uint8_t {
            NOP=0, // No operation
            APRD=1, // Auto Increment Read
            APWR=2, // Auto Increment Write
            APRW=3, // Auto Increment Read Write
            FPRD=4, // Configured Address Read
            FPWR=5, // Configured Address Write
            FPRW=6, // Configured Address Read Write
            BRD=7, // Broadcast Read
            BWR=8, // Broadcast Write
            BRW=9, // Broadcast Read Write
            LRD=10, // Logical Memory Read
            LWR=11, // Logical Memory Write
            LRW=12, // Logical Memory Read Write
            ARMW=13, // Auto Increment Read Multiple Write
            FRMW=14 // Configured Read Multiple Write
        };
        struct LenRCMBits {
          uint16_t len_           :11;
          uint16_t reserved_      :3;
          uint16_t circulating_   :1;
          uint16_t more_          :1;
        } __attribute__((packed)); // prevent structure padding
        // Note: The constructor per default calls the initializer of the first element of a union.
        union LenRCM {
          uint16_t all_;
          LenRCMBits elements_;
        };

        Command cmd_;
        uint8_t idx_;
        uint32_t address_;
        LenRCM lenRCM_;
        uint16_t irq_;

        EtherCatDatagramHeader(): cmd_(Command::NOP), idx_(0), address_(0), lenRCM_{0}, irq_(0) {}

    } __attribute__((packed)); // prevent structure padding


 public:
    EtherCatDatagramHeader header_;
    uint8_t* data_ = nullptr;
    uint16_t workingCounter_ = 0;

    // have a map of (name string => memory address) to have human-readable access to the fields in data_? would need to update this on resize(..) calls..
};


class EtherCatDatagrams {
 public:
    // Map from slave address to according Rx and TxPDOs.
    std::unordered_map<uint32_t, std::pair<EtherCatDatagram, EtherCatDatagram>> rxAndTxPdoDatagrams_;
};


} /* namespace tcan_ethercat */

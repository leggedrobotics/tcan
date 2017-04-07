/*
 * EtherCatDatagramData.hpp
 *
 *  Created on: Mar 24, 2017
 *      Author: Remo Diethelm
 */

#pragma once


namespace tcan {


class EtherCatDatagram {
 public:
    inline void resize(const uint16_t length) {
        uint8_t* oldData = data_;
        data_ = new uint8_t[length];
        std::copy(&oldData[0], &oldData[getDataLength()], data_);
        header_.lenRCM_.elements_.len_ = length;
        delete[] oldData;
    }

    template <typename T>
    inline void write(const uint16_t memoryPosition, const T& data) {
        // check if memoryPosition lies within data_?
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
        // Note: Bit field ordering depends on the Endianness which is implementation dependent.
        struct LenRCMBits {
          uint16_t len_           :11; // does this sub-byte work?
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

//        DatagramHeader(): cmd_(Command::NOP), idx_(0), address_(0), len_(0), reserved_(0), circulating_(0), more_(0), irq_(0) { }
        EtherCatDatagramHeader(): cmd_(Command::NOP), idx_(0), address_(0), lenRCM_{0}, irq_(0) { }

//        uint16_t getLength() const {
//            return (lenRCM_ & 0xFFE0) >> 5;
//        }
//
//        void setLength(uint16_t length) {
//            lenRCM_ &= ~0xFFE0; // clear bits first.
//            lenRCM_ = (length << 5) & 0xFFE0;
//        }
//
//        uint16_t getReserved() const {
//            return (lenRCM_ & 0x001C) >> 2;
//        }
//
//        void setReserved(uint16_t reserved) {
//            lenRCM_ &= ~0x001C; // clear bits first.
//            lenRCM_ = (reserved << 2) & 0x001C;
//        }
//
//        uint16_t getCirculating() const {
//            return (lenRCM_ & 0x0002) >> 1;
//        }
//
//        void setCirculating(uint16_t circulating) {
//            lenRCM_ &= ~0x0002; // clear bits first.
//            lenRCM_ = (circulating << 1) & 0x0002;
//        }
//
//        uint16_t getMore() const {
//            return (lenRCM_ & 0x0001) >> 0;
//        }
//
//        void setMore(uint16_t more) {
//            lenRCM_ &= ~0x0001; // clear bits first.
//            lenRCM_ = (more << 1) & 0x0001;
//        }
    } __attribute__((packed)); // prevent structure padding


 public:
    EtherCatDatagramHeader header_;
    uint8_t* data_ = nullptr;
    uint16_t workingCounter_ = 0;

    // have a map of (name string => memory address) to have human-readable access to the fields in data_? would need to update this on resize(..) calls..
};


class EtherCatDatagrams {
 public:
    std::unordered_map<uint32_t, std::pair<EtherCatDatagram, EtherCatDatagram>> rxAndTxPdoDatagrams_;
};


} /* namespace tcan */

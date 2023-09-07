//
//  CULWSerializer.h
//  Networked Physics Library
//
//  Created by Barry Lyu on 9/7/23.
//

#ifndef __CU_LW_SERIALIZER_H__
#define __CU_LW_SERIALIZER_H__

class LWSerializer{
private:
    std::vector<std::byte> _data;
    
public:
    LWSerializer() {}

    static std::shared_ptr<LWSerializer> alloc() {
        return std::make_shared<LWSerializer>();
    }
    
    void writeBool(bool b){
        _data.push_back(b ? std::byte(1) : std::byte(0));
    }
    
    void writeByte(std::byte b){
        _data.push_back(b);
    }
    
    void writeByteVector(const std::vector<std::byte>& v){
        _data.insert(_data.end(), v.begin(), v.end());
    }
    
    void rewriteFirstUint32(Uint32 i){
        Uint32 ii = marshall(i);
        const std::byte* bytes = reinterpret_cast<const std::byte*>(&ii);
        for (size_t j = 0; j < sizeof(Uint32); j++) {
            _data[j] = bytes[j];
        }
    }
    
    void writeFloat(float f){
        float ii = marshall(f);
        const std::byte* bytes = reinterpret_cast<const std::byte*>(&ii);
        for (size_t j = 0; j < sizeof(float); j++) {
            _data.push_back(bytes[j]);
        }
    }
    
    void writeSint32(Sint32 i){
        Sint32 ii = marshall(i);
        const std::byte* bytes = reinterpret_cast<const std::byte*>(&ii);
        for (size_t j = 0; j < sizeof(Sint32); j++) {
            _data.push_back(bytes[j]);
        }
    }
    
    void writeUint16(Uint16 i){
        Uint16 ii = marshall(i);
        const std::byte* bytes = reinterpret_cast<const std::byte*>(&ii);
        for (size_t j = 0; j < sizeof(Uint16); j++) {
            _data.push_back(bytes[j]);
        }
    }
    
    void writeUint32(Uint32 i){
        Uint32 ii = marshall(i);
        const std::byte* bytes = reinterpret_cast<const std::byte*>(&ii);
        for (size_t j = 0; j < sizeof(Uint32); j++) {
            _data.push_back(bytes[j]);
        }
    }
    
    void writeUint64(Uint64 i){
        Uint64 ii = marshall(i);
        const std::byte* bytes = reinterpret_cast<const std::byte*>(&ii);
        for (size_t j = 0; j < sizeof(Uint64); j++) {
            _data.push_back(bytes[j]);
        }
    }
    
    const std::vector<std::byte>& serialize() {
        return _data;
    }

    /**
     * Clears the input buffer.
     */
    void reset() {
        _data.clear();
    }
};

class LWDeserializer{
private:
    /** Currently loaded data */
    std::vector<std::byte> _data;
    /** Position in the data of next byte to read */
    size_t _pos;

public:
    LWDeserializer() : _pos(0) {}
    
    static std::shared_ptr<LWDeserializer> alloc() {
        return std::make_shared<LWDeserializer>();
    }
    
    void receive(const std::vector<std::byte>& msg){
        _data = msg;
        _pos = 0;
    }
    
    bool readBool(){
        if (_pos >= _data.size()) {
            return false;
        }
        uint8_t value = static_cast<uint8_t>(_data[_pos++]);
        return value == 1;
    }
    
    std::byte readByte(){
        if (_pos >= _data.size()) {
            return std::byte(0);
        }
        const std::byte b = _data[_pos++];
        return b;
    }
    
    float readFloat(){
        if (_pos >= _data.size()) {
            return 0.0f;
        }
        const float* r = reinterpret_cast<const float*>(_data.data() + _pos);
        _pos += sizeof(float);
        return marshall(*r);
    }
    
    Sint32 readSint32(){
        if (_pos >= _data.size()) {
            return 0;
        }
        const Sint32* r = reinterpret_cast<const Sint32*>(_data.data() + _pos);
        _pos += sizeof(Sint32);
        return marshall(*r);
    }
    
    Uint16 readUint16(){
        if (_pos >= _data.size()) {
            return 0;
        }
        const Uint16* r = reinterpret_cast<const Uint16*>(_data.data() + _pos);
        _pos += sizeof(Uint16);
        return marshall(*r);
    }
    
    Uint32 readUint32(){
        if (_pos >= _data.size()) {
            return 0;
        }
        const Uint32* r = reinterpret_cast<const Uint32*>(_data.data() + _pos);
        _pos += sizeof(Uint32);
        return marshall(*r);
    }
    
    Uint64 readUint64(){
        if (_pos >= _data.size()) {
            return 0;
        }
        const Uint64* r = reinterpret_cast<const Uint64*>(_data.data() + _pos);
        _pos += sizeof(Uint64);
        return marshall(*r);
    }
    
    void reset(){
        _pos = 0;
        _data.clear();
    }
    
};

#endif /* __CU_LW_SERIALIZER_H__ */

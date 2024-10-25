#include "particle_storage.hpp"
namespace systems {
    attribute_storage::attribute_storage()
		: type_size(), mask(), data(), storage_variant(storage_type::UNIT) { }

    attribute_storage::attribute_storage(
		size_t particle_capacity,
		storage_type storage_type,
		size_t type_size
	) : type_size(type_size), mask(), data(), storage_variant(storage_type) {
		switch (storage_type) {
		case storage_type::VECTOR: {
			mask.reserve(particle_capacity);
			data.reserve(particle_capacity * type_size);
			break;
		}
		case storage_type::UNIT: {
			mask.reserve(particle_capacity);
			data.~vector();
		}
		case storage_type::SHARED: {
			mask.resize(1, false);
			data.resize(type_size);
			break;
		}
		default: {
			assert(false && "unreachable: invalid storage type");
			std::exit(EXIT_FAILURE);
			break;
		}
		}
	}

    attribute_storage::attribute_storage(attribute_storage &&other)
		: type_size(other.type_size),
		mask(std::move(other.mask)),
		data(std::move(other.data)),
		storage_variant(other.storage_variant) {
		other.type_size = 0;
		other.storage_variant = storage_type::UNIT;
	}

    attribute_storage &attribute_storage::operator=(attribute_storage &&other) {
		assert(type_size == other.type_size && "error: attempted to access attribute of wrong size");
		mask = std::move(other.mask);
		data = std::move(other.data);
		storage_variant = other.storage_variant;

		other.type_size = 0;
		other.storage_variant = storage_type::UNIT;
		return *this;
    }

    void *attribute_storage::get_particle_attribute_ptr(particle_id particle) {
		assert(particle_has_attribute(particle) && "error: attempted to access non-existing attribute of particle");
		switch (storage_variant) {
		case VECTOR:
			return &data.at(particle_data_index(particle, type_size));
		case UNIT:
			return &mask.at(particle);
		case SHARED: {
			if (data.size() != type_size) {
				assert(false && "unreachable: data size mismatch, given shared storage");
				std::exit(EXIT_FAILURE);
			}
			return &data.at(0);
		}
		default: {
			assert(false && "unreachable: invalid storage type");
			std::exit(EXIT_FAILURE);
			return nullptr;
		}
		}
	}

    void *attribute_storage::set_particle_attribute_ptr(particle_id particle, void const *&&attribute) {
		switch (storage_variant) {
		case VECTOR: {
			size_t const data_index = particle_data_index(particle, type_size);
			size_t const data_size = data.size();
			bool particle_in_mask = particle < mask.size();
			bool particle_in_data = data_index + type_size <= data_size;
			if (particle_in_mask != particle_in_data) {
				assert(false && "unreachable: attribute data and mask mismatch");
				std::exit(EXIT_FAILURE);
			}
			if (!particle_in_mask) {
				size_t missing = particle + 1 - mask.size();
				mask.resize(mask.size() + missing);
			}
			mask.at(particle) = true;
			if (!particle_in_data) {
				size_t const missing_bytes = data_index + type_size - data_size;
				data.resize(data_size + missing_bytes);
			}
			return std::memcpy(&data.at(data_index), attribute, type_size);
		}
		case UNIT: {
			mask.at(particle) = true;
			return &mask.at(particle);
		}
		case SHARED: {
			if (mask.size() != 1 || data.size() != type_size) {
				assert(false && "unreachable: mask size mismatch, given shared storage");
				std::exit(EXIT_FAILURE);	
				return nullptr;
			}
			mask.at(0) = true;
			return std::memcpy(&data.at(0), attribute, type_size);
		}
		default: {
			assert(false && "unreachable: invalid storage type");
			std::exit(EXIT_FAILURE);
			return nullptr;
		}
		}
	}

    bool attribute_storage::remove_particle_attribute_ptr(particle_id particle, void *out_particle_attribute) {
		if (!particle_has_attribute(particle)) {
			return false;
		}
		switch (storage_variant) {
		case VECTOR: {
			std::memcpy(out_particle_attribute, get_particle_attribute_ptr(particle), type_size);
			mask.at(particle) = false;
			return true;
		}
		case UNIT: {
			mask.at(particle) = false;
			return true;
		}
		case SHARED: {
			if (mask.size() != 1 || data.size() != type_size) {
				assert(false && "unreachable: mask size mismatch, given shared storage");
				std::exit(EXIT_FAILURE);	
				return nullptr;
			}
			mask.at(0) = false;
			std::memcpy(out_particle_attribute, &data.at(0), type_size);
			return true;
		}
		}
	}

    bool attribute_storage::remove_particle_attribute(particle_id particle) {
		if (!particle_has_attribute(particle)) {
			return false;
		}
		switch (storage_variant) {
		case VECTOR:
		case UNIT: {
			mask.at(particle) = false;
			return true;
		}
		case SHARED: {
			if (mask.size() != 1 || data.size() != type_size) {
				assert(false && "unreachable: mask size mismatch, given shared storage");
				std::exit(EXIT_FAILURE);	
				return nullptr;
			}
			mask.at(0) = false;
			return true;
		}
		}
	}

    particle_storage::particle_storage() noexcept
		: particles(), attribute_map() { }
	particle_storage::particle_storage(size_t attribute_capacity) noexcept
		: particles(), attribute_map(attribute_capacity) {
		particles.reserve(attribute_capacity);
	}
}